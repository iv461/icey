
#pragma once

#include <boost/noncopyable.hpp>
#include <functional>
#include <icey/impl/bag_of_metaprogramming_tricks.hpp>
#include <memory>
#include <type_traits>
#include <variant>

namespace icey {
/// A special type that indicates that there is no value (note that using void causes many problems, defining an own struct is easier.)
struct Nothing {};
/// A tag to be able to recognize the following result type using std::is_base_of_v, a technique we will generally use in the following to recognize (unspecialized) class templates.
struct ResultTag {}; 
/// A Result-type like in Rust, it is a sum type that can either hold Value or Error, or, different
/// to Rust, none. 
template <class _Value, class _ErrorValue>
struct Result : public std::variant<std::monostate, _Value, _ErrorValue>, public ResultTag {
  using Value = _Value;
  using ErrorValue = _ErrorValue;
  using Self = Result<_Value, _ErrorValue>;
  static Self None() { return Result<_Value, _ErrorValue>{}; }
  static Self Ok(const _Value &x) {
    Self ret;
    ret.template emplace<1>(*x);
    return ret;
  }
  static Self Err(const _ErrorValue &x) {
    Self ret;
    ret.template emplace<2>(x);
    return ret;
  }
  bool has_value() const { return this->index() == 1; }
  bool has_error() { return this->index() == 2; }
  const Value &value() const { return std::get<1>(*this); }
  const ErrorValue &error() const { return std::get<2>(*this); }
  void set_none() { this->template emplace<0>(std::monostate{}); }
  void set_ok(const Value &x) { this->template emplace<1>(x); }
  void set_err(const ErrorValue &x) { this->template emplace<2>(x); }
};

template <class T>
constexpr bool is_result = std::is_base_of_v<ResultTag, T>;
/// Some traits of the observables
template <class T>
using obs_val = typename remove_shared_ptr_t<T>::Value;
template <class T>
using obs_err = typename remove_shared_ptr_t<T>::ErrorValue;
template <class T>
using obs_state = typename remove_shared_ptr_t<T>::State;
/// By stripping the shared_ptr, we usually get the ROS message that an observable holds
template <class T>
using obs_msg = remove_shared_ptr_t<obs_val<T>>;

namespace impl {
/// Creates a new observable of type O by passing the args to the constructor. Observables are
/// always reference counted, currently implemented with std::shared_ptr.
template <class O, typename... Args>
static std::shared_ptr<O> create_observable(Args &&...args) {
  auto observable = std::make_shared<O>(std::forward<Args>(args)...);
  return observable;
}

/// An observable. Similar to a promise in JavaScript.
template <typename _Value, typename _ErrorValue = Nothing>
class Observable : private boost::noncopyable,
                   public std::enable_shared_from_this<Observable<_Value, _ErrorValue>> {
public:
  using Value = _Value;
  using ErrorValue = _ErrorValue;
  using Self = Observable<Value, ErrorValue>;
  using MaybeValue = std::optional<Value>;
  using State = Result<Value, ErrorValue>;
  using Handler = std::function<void()>;

  bool has_error() { return value_.has_error(); }
  bool has_value() const { return value_.has_value(); }
  const Value &value() const { return value_.value(); }
  const ErrorValue &error() const { return value_.error(); }

  void register_handler(Handler cb) { handlers_.emplace_back(std::move(cb)); }

  /// TODO disable_if these hold Nothing 
  void set_value(const MaybeValue &x) {
    if (x)
      value_.set_ok(*x);
    else
      value_.set_none();
  }

  void set_error(const ErrorValue &x) { 
      value_.set_err(x); 
  }

  /// Notify about error or value, depending on the state. If there is no value, it does not notify
  void notify() {
    if constexpr(std::is_base_of_v<std::runtime_error, ErrorValue>) {
      // If we have an error and the chain stops, we re-throw the error so that we do not leave the error unhandled.
      if(this->has_error() && handlers_.empty()) { 
          //std::cout << "Unhandled promise rejection with an ErrorValue  that is an exception, re-throwing .." << std::endl;
          throw this->error();
      }
    }
    if (this->has_value() || this->has_error()) {
      for (const auto &cb : handlers_) cb();
    }
  }

  void resolve(const MaybeValue &x) {
    this->set_value(x);
    this->notify();
  }

  void reject(const ErrorValue &x) {
    this->set_error(x);
    this->notify();
  }

  template <class F>
  auto then(F &&f) {
    /// Note that it may only have errors
    static_assert(not std::is_same_v<Value, Nothing>,
                  "This observable does not have a value, so you cannot register .then() on it.");
    return std::get<0>(this->done<true>(f, true));
  }
  template <class F>
  auto except(F &&f) {
    static_assert(not std::is_same_v<ErrorValue, Nothing>,
                  "This observable cannot have errors, so you cannot register .except() on it.");
    return std::get<0>(this->done<false>(f, true));
  }

protected:
  /// Returns a function that calls the passed user callback and then writes the result in the
  /// passed output Observable (that is captured by value)
  template <bool resolve, class Output, class F>
  auto call_depending_on_signature(Output output, F &&f) {
    using FunctionArgument = std::conditional_t<resolve, Value, ErrorValue>;
    // TODO use std::invoke_result
    using ReturnType = decltype(apply_if_tuple(f, std::declval<FunctionArgument>()));
    return [output, f = std::move(f)](const FunctionArgument &x) {
      if constexpr (std::is_void_v<ReturnType>) {
        apply_if_tuple(f, x);
      } else if constexpr (is_result<ReturnType>) { 
        /// support callbacks that at runtime may return value or error
        output->value_ = apply_if_tuple(f, x);  /// TODO maybe do not overwrite whole variant but
                                                /// differentiate and set error or value
        output->notify();
      } else {  /// Other return types are interpreted as values that resolve the promise. Here, we
                /// also support returning std::optional.
        ReturnType ret = apply_if_tuple(f, x);
        output->resolve(ret);
      }
    };
  }

  /// @brief This function creates a handler for the promise and returns it as a function object.
  /// It' behvarior closely matches promises in JavaScript.
  /// It is basis for implementing the .then() function but it does three things differently:
  /// First, it does not create the new promise, the output prmise.
  /// Second, it only optionally registers this handler, it must not be registered for the graph
  /// mode. Third, we only can register on resolve or on reject, not both. on resolve is implemented
  /// such that implicitly the on reject handler passes over the error.
  /// @param this (implicit): Observable<V, E>* The input promise
  /// @param output: SharedPtr< Observable<V2, E> > or derived, the resulting promise
  /// @param f:  (V) -> V2 The user callback function to call when the input promises resolves or
  /// rejects
  template <bool resolve, class Output, class F>
  Handler create_handler(Output output, F &&f, bool register_it) {
    auto input = this->shared_from_this();  // strong reference to this for binding, instead of only
                                            // weak if we would bind this
    auto handler = [input, output,
                    call_and_resolve =
                        std::move(call_depending_on_signature<resolve>(output, f))]() {
      if constexpr (resolve) {  /// If we handle values with .then()
        if (input->has_value()) {
          call_and_resolve(input->value());
        } else if (input->has_error()) {
          output->reject(input->error());  /// Do not execute f, but propagate the error
        }
      } else {                     /// if we handle errors with .except()
        if (input->has_error()) {  /// Then only resolve with the error if there is one
          call_and_resolve(input->error());
        }  /// Else do nothing
      }
    };
    if (register_it) input->register_handler(handler);
    return handler;
  }

  /// Common function for both then and .except. bool resolve says whether f is the resolve or the
  /// reject handler (Unlike JS, we can only register one at the time)
  template <bool resolve, typename F>
  auto done(F &&f, bool register_handler) {
    /// TODO static_assert here signature for better error messages
    /// Return type depending of if the it is called when the Promise resolves or rejects
    using FunctionArgument = std::conditional_t<resolve, Value, ErrorValue>;
    using ReturnType = decltype(apply_if_tuple(f, std::declval<FunctionArgument>()));
    /// Now we want to call resolve only if it is not none, so strip optional
    using ReturnTypeSome = remove_optional_t<ReturnType>;
    if constexpr (std::is_void_v<ReturnType>) {
      auto child = create_observable<Observable<Nothing, ErrorValue>>();
      auto handler = create_handler<resolve>(child, std::move(f), register_handler);
      return std::make_tuple(child, handler);
    } else if constexpr (is_result<ReturnType>) {  /// But it may be an result type
      /// In this case we want to be able to pass over the same error
      auto child =
          create_observable<Observable<typename ReturnType::Value,
                                       typename ReturnType::ErrorValue>>();  // Must pass over error
      auto handler = create_handler<resolve>(child, std::move(f), register_handler);
      return std::make_tuple(child, handler);
    } else {  /// Any other return type V is interpreted as Result<V, Nothing>::Ok() for convenience
      /// The resulting observable always has the same ErrorValue so that it can pass through the
      /// error
      auto child = create_observable<Observable<ReturnTypeSome, ErrorValue>>();
      auto handler = create_handler<resolve>(child, std::move(f), register_handler);
      return std::make_tuple(child, handler);
    }
  }

  

  /// The last received value, it is buffered.
  State value_;
  std::vector<Handler> handlers_;
};
}  // namespace impl
}  // namespace icey