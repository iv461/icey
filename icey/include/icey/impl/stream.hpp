
#pragma once

#include <boost/noncopyable.hpp>
#include <functional>
#include <tuple>
#include <optional>
#include <memory>
#include <type_traits>
#include <variant>

namespace icey {
/// A special type that indicates that there is no value. (Using `void` for this would cause many problems,
/// so defining an extra struct is easier.)
struct Nothing {};
/// A tag to be able to recognize the following result type using std::is_base_of_v, a technique we
/// will generally use in the following to recognize (unspecialized) class templates.
struct ResultTag {};
/// A Result-type is a sum type that can either hold Value or Error, or, different
/// to Rust, none. It is used as the state for the Stream
template <class _Value, class _ErrorValue>
struct Result : private std::variant<std::monostate, _Value, _ErrorValue>, public ResultTag {
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
  bool has_none() const { return this->index() == 0; }
  bool has_value() const { return this->index() == 1; }
  bool has_error() const { return this->index() == 2; }
  const Value &value() const { return std::get<1>(*this); }
  const ErrorValue &error() const { return std::get<2>(*this); }
  void set_none() { this->template emplace<0>(std::monostate{}); }
  void set_ok(const Value &x) { this->template emplace<1>(x); }
  void set_err(const ErrorValue &x) { this->template emplace<2>(x); }
};

/// Some pattern matching for type recognition
template <class T>
struct remove_optional {
  using type = T;
};

template <class T>
struct remove_optional<std::optional<T>> {
  using type = T;
};

template <class T>
using remove_optional_t = typename remove_optional<T>::type;

template <class T>
struct remove_shared_ptr {
  using type = T;
};

template <class T>
struct remove_shared_ptr<std::shared_ptr<T>> {
  using type = T;
};

template <class T>
using remove_shared_ptr_t = typename remove_shared_ptr<T>::type;

template <class T>
struct is_tuple : std::false_type {};

template <typename... Args>
struct is_tuple<std::tuple<Args...>> : std::true_type {};

template <class T>
struct is_pair : std::false_type {};

template <typename... Args>
struct is_pair<std::pair<Args...>> : std::true_type {};

template <class T>
constexpr bool is_tuple_v = is_tuple<T>::value;

template <class T>
constexpr bool is_pair_v = is_pair<T>::value;

template <class T>
constexpr bool is_result = std::is_base_of_v<ResultTag, T>;
/// The error type of the given Stream type
template <class T>
using ValueOf = typename remove_shared_ptr_t<T>::Value;
/// The value type of the given Stream type
template <class T>
using ErrorOf = class remove_shared_ptr_t<T>::ErrorValue;
/// The ROS-message of the given Stream type
template <class T>
using MessageOf = remove_shared_ptr_t<ValueOf<T>>;

namespace impl {
/// Creates a new stream of type O by passing the args to the constructor. Streams are
/// always reference counted, currently implemented with std::shared_ptr.
template <class O, class... Args>
static std::shared_ptr<O> create_stream(Args &&...args) {
  auto stream = std::make_shared<O>(std::forward<Args>(args)...);
  return stream;
}


/// Calls the function with the given argument arg but unpacks it if it is a tuple. 
template <class Func, class Arg>
inline auto unpack_if_tuple(Func &&func, Arg &&arg) {
  if constexpr (is_tuple_v<std::decay_t<Arg>> || is_pair_v<std::decay_t<Arg>>) {
    // Tuple detected, unpack and call the function
    return std::apply(std::forward<Func>(func), std::forward<Arg>(arg));
  } else {
    // Not a tuple, just call the function directly
    return func(std::forward<Arg>(arg));
  }
}

/// \brief A stream, an abstraction over an asynchronous sequence of values.
/// It contains a state that is a Result and a list of callbacks that get notifyed when this state changes. 
///  It is conceptually very similar to a promise in JavaScript but the state transitions are not
/// final. 
/// \tparam _Value the type of the value 
/// \tparam _ErrorValue the type of the error. It can also be an exception.
/// \tparam Derived a class from which this class derives, used as an extention point.
/// \tparam DefaultDerived When new `Stream`s get created using `then` and `except`, this is used as a template parameter for `Derived` so that a default extention does not get lost when we call `then` or `except`.
///
/// References:
/// [1]https://www.boost.org/doc/libs/1_67_0/libs/fiber/doc/html/fiber/synchronization/futures/promise.html
/// [2] https://devblogs.microsoft.com/oldnewthing/20210406-00/?p=105057
/// And for a thread-safe implementation at the cost of insane complexity, see this:
/// [3] https://github.com/lewissbaker/cppcoro/blob/master/include/cppcoro/task.hpp
template <class _Value, class _ErrorValue, class Derived, class DefaultDerived>
class Stream : private boost::noncopyable,
               public Derived {
public:
  using Value = _Value;
  using ErrorValue = _ErrorValue;
  using Self = Stream<Value, ErrorValue, Derived, DefaultDerived>;
  using MaybeValue = std::optional<Value>;
  using State = Result<Value, ErrorValue>;
  using Handler = std::function<void(const State &)>;

  bool has_none() const { return state_.has_none(); }
  bool has_value() const { return state_.has_value(); }
  bool has_error() const { return state_.has_error(); }
  const Value &value() const { return state_.value(); }
  const ErrorValue &error() const { return state_.error(); }
  State &get_state() { return state_; }
  const State &get_state() const { return state_; }
  
  /// Register a handler (i.e. a callback) that gets called when the state changes. It receives the new state as an argument.
  void register_handler(Handler cb) { handlers_.emplace_back(std::move(cb)); }

  /// Sets the state to hold none, but does not notify about this state change.
  void set_none() { state_.set_none(); }
  
  /// Sets the state to hold a value, but does not notify about this state change.
  void set_value(const MaybeValue &x) {
    if (x)
      state_.set_ok(*x);
    else
      state_.set_none();
  }

  /// Sets the state to hold an error, but does not notify about this state change.
  void set_error(const ErrorValue &x) { state_.set_err(x); }

  /// Notify about the state change, but only in case it is error or value. If the state is none, it does not notify.
  /// If the state is an error and the `ErrorValue` is an exception type (a subclass of `std::runtime_error`) and also no handlers were registered, the exception is re-thrown.
  void notify() {
    if constexpr (std::is_base_of_v<std::runtime_error, ErrorValue>) {
      // If we have an error and the chain stops, we re-throw the error so that we do not leave the
      // error unhandled.
      if (this->has_error() && handlers_.empty()) {
        throw this->error();
      }
    }
    if (!this->has_none()) {
      for (const auto &cb : handlers_) cb(state_);
    }
  }

  /// Sets the state to a value and notifies.
  void put_value(const MaybeValue &x) {
    this->set_value(x);
    this->notify();
  }

  /// Sets the state to an error and notifies.
  void put_error(const ErrorValue &x) {
    this->set_error(x);
    this->notify();
  }

  template <class F>
  auto then(F &&f) {
    /// Note that it may only have errors
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, so you cannot register .then() on it.");
    return this->done<true>(f);
  }

  template <class F>
  auto except(F &&f) {
    static_assert(!std::is_same_v<ErrorValue, Nothing>,
                  "This stream cannot have errors, so you cannot register .except() on it.");
    return this->done<false>(f);
  }

protected:
  /// Returns a function that calls the passed user callback and then writes the result in the
  /// passed output Stream (that is captured by value)
  template <class Output, class F>
  auto call_depending_on_signature(Output output, F &&f) {
    return [output, f = std::forward<F>(f)](const auto &x) {
      using ReturnType = decltype(unpack_if_tuple(f, x));
      if constexpr (std::is_void_v<ReturnType>) {
        unpack_if_tuple(f, x);
      } else if constexpr (is_result<ReturnType>) {
        /// support callbacks that at runtime may return value or error
        output->state_ = unpack_if_tuple(f, x);
        output->notify();
      } else {  /// Other return types are interpreted as values that are put into the stream. Here, we
                /// also support returning std::optional.
        ReturnType ret = unpack_if_tuple(f, x);
        output->put_value(ret);
      }
    };
  }

  /// @brief This function creates a handler for the Steam and returns it as a function object.
  /// It' behvarior closely matches promises in JavaScript.
  /// It is basis for implementing the .then() function but it does two things differently:
  /// First, it does not create the new Strean, it receives an already created "output"-Stream.
  /// Second, we can only register a "onResolve"- or an "onReject"-handler, not both. But
  /// "onResolve" is already implemented such that implicitly the "onReject"-handler passes over the
  /// error, meaning the onReject-handler is (_, Err) -> (_, Err): x, the identity function.
  /// @param this (implicit): Stream<V, E>* The input
  /// @param output: SharedPtr< Stream<V2, E> >, the ouput Stream where the result is written in
  /// @param f:  (V) -> V2 The user callback function to call when the input Stream receives a value or
  /// an error
  template <bool put_value, class Output, class F>
  void create_handler(Output output, F &&f) {
    auto handler = [output, call_and_put_value =
                        std::move(call_depending_on_signature(output, f))](const State &state) {
      if constexpr (put_value) {  /// If we handle values with .then()
        if (state.has_value()) {
          call_and_put_value(state.value());
        } else if (state.has_error()) {
          output->put_error(state.error());  /// Do not execute f, but propagate the error
        }
      } else {                     /// if we handle errors with .except()
        if (state.has_error()) {  /// Then only put_value with the error if there is one
          call_and_put_value(state.error());
        }  /// Else do nothing
      }
    };
    this->register_handler(handler);
  }

  /// Common function for both .then and .except. The template argument "put_value" says whether f is
  /// the put_value or the put_error handler (Unlike JS, we can only register one at the time, this is
  /// for better code generation)
  template <bool put_value, class F>
  auto done(F &&f) {
    /// Return type depending of if the it is called when the Promise put_values or put_errors
    using FunctionArgument = std::conditional_t<put_value, Value, ErrorValue>;
    /// Only if we put_value we pass over the error. except does not pass the error, only the handler may create a new error
    using NewError = std::conditional_t<put_value, ErrorValue, Nothing>;
    using ReturnType = decltype(unpack_if_tuple(f, std::declval<FunctionArgument>()));
    /// Now we want to call put_value only if it is not none, so strip optional
    using ReturnTypeSome = remove_optional_t<ReturnType>;
    if constexpr (std::is_void_v<ReturnType>) {
      auto child = create_stream<Stream<Nothing, NewError, DefaultDerived, DefaultDerived>>();
      create_handler<put_value>(child, std::forward<F>(f));
      return child;
    } else if constexpr (is_result<ReturnType>) {  /// But it may be an result type
      /// In this case we want to be able to pass over the same error
      auto child =
          create_stream<Stream<typename ReturnType::Value,
                                   typename ReturnType::ErrorValue, 
                                   DefaultDerived, DefaultDerived>>();  // Must pass over error
      create_handler<put_value>(child, std::forward<F>(f));
      return child;
    } else {  /// Any other return type V is interpreted as Result<V, Nothing>::Ok() for convenience
      /// The resulting stream always has the same ErrorValue so that it can pass through the
      /// error
      auto child = create_stream<Stream<ReturnTypeSome, NewError,
                                DefaultDerived, DefaultDerived>>();
      create_handler<put_value>(child, std::forward<F>(f));
      return child;
    }
  }

  State state_;
  std::vector<Handler> handlers_;
};
}  // namespace impl
}  // namespace icey