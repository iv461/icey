#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <tuple>
#include <type_traits>
#include <variant>

#include <unordered_set> 

namespace icey {
/// A special type that indicates that there is no value. (Using `void` for this would cause many
/// problems, so defining an extra struct is easier.)
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
    ret.template emplace<1>(x);
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
struct is_optional : std::false_type {};

template <class T>
struct is_optional<std::optional<T>> : std::true_type {};

template <class T>
constexpr bool is_optional_v = is_optional<T>::value;

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

/// The unit tests enable tracking the allocation of Streams and then assert in each unit-test that all Streams are freed if the ROS node is freed, i.e. no memory leak occured.
#ifdef ICEY_DEBUG_TRACK_STREAM_ALLOCATIONS
extern std::unordered_set<void*> g_impls;
#endif

/// Creates a new stream of type O by passing the args to the constructor. Streams are
/// always reference counted, currently implemented with std::shared_ptr.
template <class O, class... Args>
static std::shared_ptr<O> create_stream(Args &&...args) {
  auto stream = std::make_shared<O>(std::forward<Args>(args)...);
#ifdef ICEY_DEBUG_TRACK_STREAM_ALLOCATIONS
  g_impls.emplace(stream.get());
#ifdef ICEY_DEBUG_PRINT_STREAM_ALLOCATIONS
  std::cout << "Added impl 0x" << std::size_t(stream.get()) << " to g_impls" << std::endl;
#endif  
#endif
  return stream;
}

/// Calls the function with the given argument arg but unpacks it if it is a tuple.
template <class F, class Arg>
inline auto unpack_if_tuple(F &&f, Arg &&arg) {
  if constexpr (is_tuple_v<std::decay_t<Arg>> || is_pair_v<std::decay_t<Arg>>) {
    // Tuple detected, unpack and call the function
    return std::apply(std::forward<F>(f), std::forward<Arg>(arg));
  } else {
    // Not a tuple, just call the function directly
    return std::forward<F>(f)(std::forward<Arg>(arg));
  }
}

struct StreamImplBase {};
/// \brief A stream, an abstraction over an asynchronous sequence of values.
/// It has a state of type Result and a list of callbacks that get notified when this state changes.
///  It is conceptually very similar to a promise in JavaScript but the state transitions are not
/// final.
/// \tparam _Value the type of the value
/// \tparam _ErrorValue the type of the error. It can also be an exception.
/// \tparam Base a class from which this class derives, used as an extention point.
/// \tparam DefaultBase When new `Stream`s get created using `then` and `except`, this is used as
/// a template parameter for `Base` so that a default extention does not get lost when we call
/// `then` or `except`.
///
template <class _Value, class _ErrorValue, class Base, class DefaultBase>
class Stream : public StreamImplBase, public Base {
public:
  using Value = _Value;
  using ErrorValue = _ErrorValue;
  using Self = Stream<Value, ErrorValue, Base, DefaultBase>;
  using State = Result<Value, ErrorValue>;

  /// If no error is possible (ErrorValue is Nothing), this it just the Value instead of the State
  /// to not force the user to write unnecessary error handling/unwraping code.
  using MaybeResult = std::conditional_t<std::is_same_v<ErrorValue, Nothing>, Value, State>;
  using Handler = std::function<void(const State &)>;

  Stream() = default;
  /// A Stream is non-copyable since it has members that reference it and therefore it should change it's adress.
  Stream(const Self &) = delete;
  /// A Stream is non-movable since it has members that reference it and therefore it should change it's adress.
  Stream(Self &&) = delete;
  /// A Stream is non-copyable since it has members that reference it and therefore it should change it's adress.
  Stream &operator=(const Self &) = delete;
  /// A Stream is non-movable since it has members that reference it and therefore it should change it's adress.
  Stream &operator=(Self &&) = delete;

#ifdef ICEY_DEBUG_TRACK_STREAM_ALLOCATIONS
  ~Stream()  {
    if(g_impls.contains(this)) {
#ifdef ICEY_DEBUG_PRINT_STREAM_ALLOCATIONS
      std::cout << "Destructed and erased 0x" << std::hex << std::size_t(this) << " from g_impls" << std::endl;
#endif
      g_impls.erase(this);
    }
  }
#else 
  ~Stream() = default;
#endif 

  bool has_none() const { return state_.has_none(); }
  bool has_value() const { return state_.has_value(); }
  bool has_error() const { return state_.has_error(); }
  const Value &value() const { return state_.value(); }
  const ErrorValue &error() const { return state_.error(); }
  State &get_state() { return state_; }
  const State &get_state() const { return state_; }


  /// Register a handler (i.e. a callback) that gets called when the state changes. It receives the
  /// new state as an argument.
  void register_handler(Handler &&cb) { handlers_.emplace_back(std::move(cb)); }

  /// Sets the state to hold none, but does not notify about this state change.
  void set_none() { state_.set_none(); }

  /// Sets the state to hold a value, but does not notify about this state change.
  void set_value(const Value &x) { state_.set_ok(x); }

  /// Sets the state to hold an error, but does not notify about this state change.
  void set_error(const ErrorValue &x) { state_.set_err(x); }

  /// Returns the current state and sets it to none.
  State take_state() {
    auto current_state = state_;
    this->set_none();
    return current_state;
  }

  /// Returns the current state and sets it to none. If no error is possible (ErrorValue is not
  /// Nothing), it just the Value to not force the user to write unnecessary error
  /// handling/unwraping code.
  MaybeResult take() {
    auto current_state = this->take_state();
    if constexpr (std::is_same_v<ErrorValue, Nothing>) {
      return current_state.value();
    } else {
      return current_state;
    }
  }

  /// It takes (calls take) the current state and notifies the callbacks. It notifies only in case
  /// we have an error or value. If the state is none, it does not notify. If the state is an error
  /// and the `ErrorValue` is an exception type (a subclass of `std::runtime_error`) and also no
  /// handlers were registered, the exception is re-thrown.
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
  void put_value(const Value &x) {
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
    return this->done<true>(std::forward<F>(f));
  }

  template <class F>
  auto except(F &&f) {
    static_assert(!std::is_same_v<ErrorValue, Nothing>,
                  "This stream cannot have errors, so you cannot register .except() on it.");
    return this->done<false>(std::forward<F>(f));
  }

protected:
  /// Returns a function that calls the passed user callback and then writes the result in the
  /// passed output Stream (that is captured by value)
  template <class Output, class F>
  static void call_depending_on_signature(const auto &x, Output output, F &f) {    
    using ReturnType = decltype(unpack_if_tuple(f, x));
    if constexpr (std::is_void_v<ReturnType>) {
      unpack_if_tuple(f, x);
    } else if constexpr (is_result<ReturnType>) {
      /// support callbacks that at runtime may return value or error
      output->state_ = unpack_if_tuple(f, x);
      output->notify();
    } else if constexpr (is_optional_v<ReturnType>) {
      auto ret = unpack_if_tuple(f, x);
      if (ret) output->put_value(*ret);
    } else {  /// Other return types are interpreted as values that are put into the stream.
      ReturnType ret = unpack_if_tuple(f, x);
      output->put_value(ret);
    }
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
  /// @param f:  (V) -> V2 The user callback function to call when the input Stream receives a value
  /// or an error
  template <bool put_value, class Output, class F>
  void create_handler(Output output, F &&f) {
    this->register_handler([output, f](const State &state) {
      if constexpr (put_value) {  /// If we handle values with .then()
        if (state.has_value()) {
          call_depending_on_signature(state.value(), output, f);
        } else if (state.has_error()) {
          output->put_error(state.error());  /// Do not execute f, but propagate the error
        }
      } else {                    /// if we handle errors with .except()
        if (state.has_error()) {  /// Then only put_value with the error if there is one
          call_depending_on_signature(state.error(), output, f);
        }  /// Else do nothing
      }
    });
  }

  /// Common function for both .then and .except. The template argument "put_value" says whether f
  /// is the put_value or the put_error handler (Unlike JS, we can only register one at the time,
  /// this is for better code generation)
  template <bool put_value, class F>
  auto done(F &&f) {
    /// Return type depending of if the it is called when the Promise put_values or put_errors
    using FunctionArgument = std::conditional_t<put_value, Value, ErrorValue>;
    /// Only if we put_value we pass over the error. except does not pass the error, only the
    /// handler may create a new error
    using NewError = std::conditional_t<put_value, ErrorValue, Nothing>;
    using ReturnType = decltype(unpack_if_tuple(f, std::declval<FunctionArgument>()));
    /// Now we want to call put_value only if it is not none, so strip optional
    using ReturnTypeSome = remove_optional_t<ReturnType>;
    if constexpr (std::is_void_v<ReturnType>) {
      auto output = create_stream<Stream<Nothing, NewError, DefaultBase, DefaultBase>>();
      create_handler<put_value>(output, std::forward<F>(f));
      return output;
    } else if constexpr (is_result<ReturnType>) {  /// But it may be an result type
      /// In this case we want to be able to pass over the same error
      auto output =
          create_stream<Stream<typename ReturnType::Value, typename ReturnType::ErrorValue,
                               DefaultBase, DefaultBase>>();  // Must pass over error
      create_handler<put_value>(output, std::forward<F>(f));
      return output;
    } else {  /// Any other return type V is interpreted as Result<V, Nothing>::Ok() for convenience
      /// The resulting stream always has the same ErrorValue so that it can pass through the
      /// error
      auto output =
          create_stream<Stream<ReturnTypeSome, NewError, DefaultBase, DefaultBase>>();
      create_handler<put_value>(output, std::forward<F>(f));
      return output;
    }
  }

  State state_;
  std::vector<Handler> handlers_;
};
}  // namespace impl
}  // namespace icey