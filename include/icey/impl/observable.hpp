
#pragma once 

#include <variant> 
#include <functional>
#include <type_traits>
#include <memory> 

#include <boost/type_index.hpp>
#include <boost/noncopyable.hpp>
#include <icey/impl/bag_of_metaprogramming_tricks.hpp>

namespace icey {
struct Nothing {}; 
/// A Result-type like in Rust, it is a sum type that can either hold Value or Error, or, different to Rust, none.
/// We create a new type to be able to recognize it when it is returned from a user callback.
struct ResultBase{};
template <class _Value, class _ErrorValue>
struct Result : public std::variant<std::monostate, _Value, _ErrorValue>, public ResultBase {
  using Value = _Value;
  using ErrorValue =  _ErrorValue;
  using Self = Result<_Value, _ErrorValue>;
  static Self None() {return Result<_Value, _ErrorValue>{};}
  static Self Ok(const _Value &x) { Self ret; ret.template emplace<1>(*x); return ret;}
  static Self Err(const _ErrorValue &x) { Self ret; ret.template emplace<2>(x); return ret;}
  bool has_error() { return this->index() == 2; }
  bool has_value() const { return this->index() == 1; }
  const Value &value() const { return std::get<1>(*this); }
  const ErrorValue &error() const { return std::get<2>(*this); }
};

/*template<class T, class V, class E>
constexpr bool is_result = std::is_same_v<T, Result<V, E>>;*/
template<class T>
constexpr bool is_result = std::is_base_of_v<ResultBase, T>;

/// Some traits of the observables
template<class T> 
using obs_val = typename remove_shared_ptr_t<T>::Value;
template<class T> 
using obs_err = typename remove_shared_ptr_t<T>::ErrorValue;
template<class T> 
using obs_state = typename remove_shared_ptr_t<T>::State;

/*
template <typename... Args>
struct observable_traits {
  static_assert(
      (is_shared_ptr<Args> && ...),
      "The arguments must be a shared_ptr< icey::Observable >, but it is not a shared_ptr");
  static_assert((std::is_base_of_v<ObservableBase, remove_shared_ptr_t<Args>> && ...),
                "The arguments must be an icey::Observable");
};
*/

/// Creates a new observable of type O by passing the args to the constructor. Observables are always reference counted by using shared pointers
template <class O, typename... Args>
static std::shared_ptr<O> create_observable(Args &&...args) {
    auto observable = std::make_shared<O>(std::forward<Args>(args)...);
    //observable->class_name = boost::typeindex::type_id_runtime(*observable).pretty_name();
    return observable;
}

class Context;

namespace impl {
/// An observable. Similar to a promise in JavaScript.
/// TODO consider CRTP, would also be beneficial for PIMPL
template <typename _Value, typename _ErrorValue = Nothing>
class Observable : private boost::noncopyable, public std::enable_shared_from_this<Observable<_Value, _ErrorValue>> {
  friend class Context; 
public:
  using Value = _Value;
  using ErrorValue =  _ErrorValue;
  using Self = Observable<Value, ErrorValue>;
  using MaybeValue = std::optional<Value>;
  using State = Result<Value, ErrorValue>;
  using Handler = std::function<void()>;

  virtual bool has_error() { return value_.has_error(); }
  virtual bool has_value() const { return value_.has_value(); }

  virtual const Value &value() const { return value_.value(); }
  virtual const ErrorValue &error() const { return value_.error(); }

  void _register_handler(Handler cb) {handlers_.emplace_back(std::move(cb)); }

  /// Set without notify
  /// TODO this will destroy first, see behttps://stackoverflow.com/a/78894559
  void _set_value(const MaybeValue &x) { 
      if(x) 
        value_.template emplace<1>(*x);
      else 
        value_.template emplace<0>(std::monostate{});
  }
  void _set_error(const ErrorValue &x) { value_.template emplace<2>(x); }


  /// Notify about error or value, depending on the state. If there is no value, it does not notify
  void _notify() {
    //if (icey_debug_print)
      //std::cout << "[" + this->class_name + ", " + this->name + "] _notify was called" << std::endl;
    //if (run_graph_engine_) 
      //run_graph_engine_(this->index.value());
  
    if(this->has_value() || this->has_error()) {
      for (auto cb : handlers_) cb();
    }
  }

  void resolve(const MaybeValue &x) {   
    this->_set_value(x);
    this->_notify();
  }

  void reject(const ErrorValue &x) {
    this->_set_error(x);
    this->_notify();
  }

  /// @brief Returns the given function, but if called with a tuple, the tuple is first unpacked and passed as multiple arguments
  /// TODO use hana::unpack
  /// @param f an arbitrary callable
  template <class F>
  static auto apply_if_needed(F && f) {
    return[f = std::move(f)](const auto &x){
      using ReturnType = decltype(apply_if_tuple(f, x));
      if constexpr (std::is_void_v<ReturnType>) {
        apply_if_tuple(f, x);
      } else {
        return apply_if_tuple(f, x);
      }
    };
  }

  /// @brief This function creates a handler for the promise and returns it as a function object. 
  /// It is basis for implementing the .then() function but it does three things differently: 
  /// First, it does not create the new promise, the output prmise. 
  /// Second, it only optionally registers this handler, it must not be registered for the graph mode.
  /// Third, we only can register on resolve or on reject, not both. on resolve is implemented such that implicitly the on reject handler 
  /// passes over the error. 
  /// @param this (implicit): Observable<V, E>* The input promise
  /// @param output: SharedPtr< Observable<V2, E> > or derived, the resulting promise
  /// @param f:  (V) -> V2 The user callback function to call when the input promises resolves or rejects
  template <bool resolve, class Output, class F>
  Handler create_handler(Output output, F &&f, bool register_it) {
    auto input = this->shared_from_this(); // strong reference to this for binding, instead of only weak if we would bind this
    //observable_traits<Output>{}; TODO 
    using FunctionArgument = std::conditional_t<resolve, Value, ErrorValue >;
    // TODO use std::invoke_result
    using ReturnType = decltype(apply_if_tuple(f, std::declval< FunctionArgument >()));
    //const auto on_new_value = apply_if_needed(f);
    /// TODO refactor, bind
    auto notify = [output]() { output->_notify(); };
    auto call_and_resolve = [output, f = std::move(f)](const FunctionArgument &x) {
        if constexpr (std::is_void_v<ReturnType>) {
            apply_if_tuple(f, x);
        } else if constexpr(is_result<ReturnType>) { /// support callbacks that at runtime may return value or error
          output->value_ = apply_if_tuple(f, x); /// TODO maybe do not overwrite whole variant but differentiate and set error or value
        } else { /// support returning std::optional
          ReturnType result = apply_if_tuple(f, x);
          output->_set_value(result); /// Do not notify, only set (may be none)
        }
    };
    
    auto resolve_if_needed = [input, output, call_and_resolve=std::move(call_and_resolve)]() {
      if constexpr (!resolve) { /// If we .catch() 
        if(input->has_error())  { /// Then only resolve with the error if there is one
          call_and_resolve(input->error());
        } /// Else do nothing
      } else {
        if(input->has_value()) { 
          call_and_resolve(input->value());
        } else if (input->has_error()) {
          output->_set_error(input->error()); /// Important: do not execute computation, but propagate the error
        }
      }
    };

    auto handler = [=]() { resolve_if_needed(); notify(); };//hana::compose(notify, compute, get_input_value);
    if (register_it) 
      input->_register_handler(handler);
    return handler;
  }

  /// TODO return handler
  template <bool resolve, bool register_handler, typename F>
  auto done(F &&f) {
    /// TODO static_assert here signature for better error messages
    /// Return type depending of if the it is called when the Promise resolves or rejects
    using FunctionArgument = std::conditional_t<resolve, Value, ErrorValue >;
    using ReturnType = decltype(apply_if_tuple(f, std::declval< FunctionArgument >()));
    /// Now we want to call resolve only if it is not none, so strip optional
    using ReturnTypeSome = remove_optional_t<ReturnType>;
    if constexpr (std::is_void_v<ReturnType>) {
      /// create a dummy to satisfy the graph
      auto child = create_observable< Observable<Nothing, ErrorValue> >(); // Must pass over error 
      auto handler = create_handler<resolve>(child, std::move(f), register_handler);
      /// return nothing so that no computation can be made based on the result
    } else if constexpr (is_result<ReturnType>) { /// But it may be an result type
      /// In this case we want to be able to pass over the same error 
      auto child = create_observable< Observable< typename ReturnType::Value, typename ReturnType::ErrorValue> >(); // Must pass over error 
      auto handler = create_handler<resolve>(child, std::move(f), register_handler);
      return child;
    } else { /// Any other return type V is interpreted as Result<V, Nothing>::Ok() for convenience
      /// The resulting observable always has the same ErrorValue so that it can pass through the error
      auto child = create_observable< Observable<ReturnTypeSome, ErrorValue > >();
      auto handler = create_handler<resolve>(child, std::move(f), register_handler);
      return child;
    }
  }  

    template <class F> auto then(F &&f) { return this->done<true, true>(f); }

    template <class F> 
    auto except(F &&f) { 
        static_assert(not std::is_same_v < ErrorValue, Nothing >, "This observable cannot have errors, so you cannot register .except() on it.");
        return this->done<false, true>(f); 
    }


  /// The last received value, it is buffered. It is buffered only to be able to do graph mode.
  State value_; 
  std::vector<Handler> handlers_;
};
}
}