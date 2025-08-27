/// Copyright © 2025 Technische Hochschule Augsburg
/// All rights reserved.
/// Author: Ivo Ivanov
/// This software is licensed under the Apache License, Version 2.0.

#pragma once

#include <boost/hana.hpp>
#include <boost/hana/ext/std/tuple.hpp>  /// Needed so that we can use std::tuple instead of custom hana tuples everywhere: https://stackoverflow.com/a/34318002
#include <boost/noncopyable.hpp>
#include <boost/type_index.hpp>
#include <coroutine>
#include <functional>
#include <icey/icey_async_await.hpp>
#include <icey/impl/field_reflection.hpp>
#include <icey/impl/stream.hpp>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <tuple>
#include <unordered_map>

/// TF2 support:
#include "tf2_ros/message_filter.h"
//#include "tf2_ros/transform_broadcaster.h"
//#include "tf2_ros/transform_listener.h"
// Message filters library: (.h so that this works with humble as well)
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace icey {

#ifdef ICEY_DEGUG_RECORD_THREAD_IDS
extern std::unordered_set<std::thread::id> g_used_thread_ids;
#define ICEY_DEGUG_TRACE_THIS_THREAD_ID g_used_thread_ids.emplace(std::this_thread::get_id());
#else
#define ICEY_DEGUG_TRACE_THIS_THREAD_ID
#endif

template <class T>
struct t_is_shared_ptr : std::false_type {};

template <class T>
struct t_is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <class T>
constexpr bool is_shared_ptr = t_is_shared_ptr<T>::value;

template <typename T, typename... Ts>
concept AnyOf = (std::is_convertible_v<T, Ts> || ...);

namespace hana = boost::hana;

inline bool icey_debug_print = false;

/// Returns a string that represents the type of the given value: i.e. "icey::Stream<int>"
template <class T>
static std::string get_type(T &t) {
  std::stringstream ss;
  auto this_class = boost::typeindex::type_id_runtime(t).pretty_name();
  ss << "[" << this_class << " @ 0x" << std::hex << size_t(&t) << "]";
  return ss.str();
}

/// A tag to be able to recognize the type "Stream", all types deriving from StreamTag satisfy the
/// `AnyStream` concept. \sa AnyStream
struct StreamTag {};

template <class T>
constexpr bool is_stream = std::is_base_of_v<StreamTag, T>;

/// A stream type with any error or value type. \sa StreamTag
template <class T>
concept AnyStream = std::is_base_of_v<StreamTag, T>;

/// A stream type is error-free, meaning its Error is Nothing. It's value should not be Nothing
/// because this does not make any sense.
template <class T>
concept ErrorFreeStream =
    AnyStream<T> && std::is_same_v<ErrorOf<T>, Nothing> && !std::is_same_v<ValueOf<T>, Nothing>;

template <class V>
struct Validator;
template <class V>
struct ParameterStream;
template <class Value>
struct ValueOrParameter;
template <class V>
struct SubscriptionStream;
struct TimerStream;
template <class V>
struct PublisherStream;
template <class V>
struct ServiceStream;
template <class V>
struct TimeoutFilter;
struct TransformBuffer;
struct TransformSubscriptionStream;
template <class V>
struct Buffer;
template <class V>
struct TransformSynchronizer;
struct TransformPublisherStream;

/// The context is what is returned when calling `node->icey()` (NodeWithIceyContext::icey).
/// It provides an new node-like API for creating ROS entities, these entities are however streams.
//  It also provides bookkeeping i.e. holding the shared pointers to subscriptions/timers/publishers
//  etc., so that you do not have to do it.
class Context : public ContextAsyncAwait,
                  private boost::noncopyable {
public:
  /// The parameter validation function that allows the parameter update if the returned error
  /// string is empty (i.e. "") and rejects it otherwise with this non-empty error string.
  using FValidate = std::function<std::string(const rclcpp::Parameter &)>;
  ///
  template <class V>
  using GetValue = std::function<V()>;

  /// Constructs the Context from the given node pointer. Supports both rclcpp::Node as well as a
  /// lifecycle node.
  /// @param node the node
  /// @tparam NodeT rclcpp::Node or rclcpp_lifecycle::LifecycleNode
  template <class NodeT>
  explicit Context(NodeT *node) : ContextAsyncAwait(node) {}

  /// Declares a single parameter to ROS and register for updates. The ParameterDescriptor is
  /// created automatically matching the given Validator.
  /// \tparam ParameterT The type of the parameter. ROS actually allows for type changes at runtime
  /// but we do not want that. \param parameter_name name \param default_value Every parameter must
  /// be initialized, this value will be used for this. \param validator A validator constraints the
  /// parameter to certain values if provided \param description Optionally, a description of the
  /// parameter may be provided \param read_only If set to true, parameter updates at runtime will
  /// be rejected \param ignore_override See rclcpp::Node::declare_parameter documentation \sa For
  /// more detailed documentation:
  /// [rclcpp::Node::declare_parameter](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4I0EN6rclcpp4Node17declare_parameterEDaRKNSt6stringERK10ParameterTRKN14rcl_interfaces3msg19ParameterDescriptorEb)
  template <class ParameterT>
  ParameterStream<ParameterT> declare_parameter(const std::string &parameter_name,
                                                const ParameterT &default_value,
                                                const Validator<ParameterT> &validator = {},
                                                std::string description = "",
                                                bool read_only = false,
                                                bool ignore_override = false);

  // clang-format off
  /*!
  \brief Declare a given parameter struct to ROS.
  \tparam ParameterStruct the type of the parameter struct. It must be a struct/class with fields of
  either a primitive type supported by ROS (e.g. `double`) or a `icey::Parameter`, or another
  (nested) struct with more such fields.

  \param params The parameter struct object where the values will be written to.
  \param notify_callback The callback that gets called when any field changes (optional)
  \param name_prefix Prefix for each parameter (optional). Used by the recursive call to support nested structs. 
  \note The passed parameter struct object `params` must have the same lifetime as the node, so it's best is to
  store it as a member of the node class.

  Example usage:
  \verbatim
    /// Here you declare in a single struct all parameters of the node:
    struct NodeParameters {
      /// We can have regular fields :
      double amplitude{3};

      /// And as well parameters with constraints and a description:
      icey::Parameter<double> frequency{10., icey::Interval(0., 25.),
                                          std::string("The frequency of the sine")};

      icey::Parameter<std::string> mode{"single", icey::Set<std::string>({"single", "double", "pulse"})};

      /// We can also have nested structs with more parameters, they will be named others.max_amp, others.cov: 
      struct OtherParams { 
        double max_amp = 6.; 
        std::vector<double> cov; 
      } others;
    };

    class MyNode : public icey::Node {
      MyNode(std::string name): icey::Node(name) {
          /// Now simply declare the parameter struct and a callback that is called when any field updates: 
          this->icey().declare_parameter_struct(params_, [&](const std::string &changed_parameter) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Parameter " << changed_parameter << " changed");
          });
      }
      // Hold the parameter struct inside the class:
      NodeParameters params_;
    };
  \endverbatim
  */
  // clang-format on
  template <class ParameterStruct>
  void declare_parameter_struct(
      ParameterStruct &params, const std::function<void(const std::string &)> &notify_callback = {},
      std::string name_prefix = "");

  /// Create a subscription stream.
  /// Works otherwise the same as [rclcpp::Node::create_subscription].
  template <class MessageT>
  SubscriptionStream<MessageT> create_subscription(
      const std::string &name, const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
      const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions());

  /// Create a subscription stream and registers the given callback. The callback can be either
  /// synchronous or asynchronous. Works otherwise the same as [rclcpp::Node::create_subscription].
  template <class MessageT, class Callback>
  SubscriptionStream<MessageT> create_subscription(
      const std::string &name, Callback &&cb, const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
      const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions());

  /// Create a subscription that subscribes to a single transform between two frames.
  /// This stream will emit a value every time the transform between these two frames changes, i.e.
  /// it is a stream. If you need to lookup transforms at a specific point in time, look instead at
  /// `Context::create_transform_buffer`.
  TransformSubscriptionStream create_transform_subscription(
      ValueOrParameter<std::string> target_frame, ValueOrParameter<std::string> source_frame);

  /// Create a publisher stream.
  /// Works otherwise the same as [rclcpp::Node::create_publisher].
  template <class Message>
  PublisherStream<Message> create_publisher(const std::string &topic_name,
                                            const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
                                            const rclcpp::PublisherOptions publisher_options = {});

  /// Create a publisher to publish transforms on the `/tf` topic. Works otherwise the same as
  /// [tf2_ros::TransformBroadcaster].
  TransformPublisherStream create_transform_publisher();

  /// Declares a parameter and registers a validator callback and a callback that will get called
  /// when the parameters updates.
  template <class ParameterT, class CallbackT>
  auto add_parameter(const std::string &name, const ParameterT &default_value,
                     CallbackT &&update_callback,
                     const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor = {},
                     FValidate f_validate = {}, bool ignore_override = false) {
    parameter_validators_.emplace(name, f_validate);
    add_parameter_validator_if_needed();
    auto param = node_parameters_->declare_parameter(name, rclcpp::ParameterValue(default_value),
                                                     parameter_descriptor, ignore_override);
    auto param_subscription =
        std::make_shared<rclcpp::ParameterEventHandler>(static_cast<NodeBase &>(*this));
    auto cb_handle =
        param_subscription->add_parameter_callback(name, std::forward<CallbackT>(update_callback));
    parameters_.emplace(name, std::make_pair(param_subscription, cb_handle));
    return param;
  }

  /// Subscribe to a transform on tf between two frames
  template <class OnTransform, class OnError>
  auto add_tf_subscription(GetValue<std::string> target_frame, GetValue<std::string> source_frame,
                           OnTransform &&on_transform, OnError &&on_error) {
    add_tf_listener_if_needed();
    tf2_listener_->add_subscription(target_frame, source_frame, on_transform, on_error);
    return tf2_listener_;
  }

  auto add_tf_broadcaster_if_needed() {
    if (!tf_broadcaster_)
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_base());
    return tf_broadcaster_;
  }

  std::shared_ptr<TFListener> add_tf_listener_if_needed() {
    if (!tf2_listener_) {
      /// We need only one subscription on /tf, but we can have multiple transforms on which we
      /// listen to
      tf2_listener_ = std::make_shared<TFListener>(node_base());
    }
    return tf2_listener_;
  }

  /// Creates a new stream of type S by passing the args to the constructor. It adds the impl to the
  /// list of streams so that it does not go out of scope. It also sets the context.
  template <AnyStream S, class... Args>
  S create_stream(Args &&...args) {
    S stream(*this, std::forward<Args>(args)...);
    /// Track (i.e. reference) the Stream impl so that it does not go out of scope.
    stream.impl()->context = this->shared_from_this();
    return stream;
  }

  /// Creates a new stream impl and adds it to the list of stream impls so that it does not go out
  /// of scope.
  template <class StreamImpl>
  Weak<StreamImpl> create_stream_impl() {
    auto impl = impl::create_stream<StreamImpl>();
    stream_impls_.push_back(impl);
    return impl;
  }

  /// Get the NodeBase, i.e. the ROS node using which this Context was created.
  NodeBase &node_base() { return static_cast<NodeBase &>(*this); }

protected:
  /// Installs the validator callback
  void add_parameter_validator_if_needed() {
    if (validate_param_cb_) return;
    this->validate_param_cb_ = node_parameters_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          for (const auto &parameter : parameters) {
            /// We want to skip validating parameters that we didn't declare, for example here we
            /// are getting called for parameters like "qos_overrides./tf.publisher.durability"
            if (parameter_validators_.contains(parameter.get_name())) {
              result.reason = parameter_validators_.at(parameter.get_name())(parameter);
            }
          }
          result.successful = result.reason == "";
          return result;
        });
  }

  /// A map that stores for each parameter name some ROS entities that we need to hold to be able to
  /// receive parameter updates.
  std::unordered_map<std::string, std::pair<std::shared_ptr<rclcpp::ParameterEventHandler>,
                                            std::shared_ptr<rclcpp::ParameterCallbackHandle>>>
      parameters_;

  /// Parameter validators for each parameter name, containing the values of a parameter.
  std::unordered_map<std::string, FValidate> parameter_validators_;

  /// Callback for validating a list of parameter.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_param_cb_;
  /// TF Support
  std::shared_ptr<TFListener> tf2_listener_;
  /// This is a simple wrapper around a publisher that publishes on /tf. There is really nothing
  /// interesting under the hood of tf2_ros::TransformBroadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// All the streams that were created, they correspond mostly to one ROS entity like sub/pub/timer
  /// etc.
  std::vector<std::shared_ptr<impl::StreamImplBase>> stream_impls_;
};

/// The default augmentation of a stream implementation: The context and a timeout.
class StreamImplDefault {
public:
  StreamImplDefault() {}
  /// A weak reference to the Context, it is needed so that Streams can create more streams that
  /// need access to the ROS node, i.e. via `.publish`.
  Weak<Context> context;

  /// The coroutine that will be called once after this stream has a value
  std::coroutine_handle<> continuation_;
  bool registered_continuation_callback_{false};
};

/// Adds a default extention inside the impl::Stream by default.
/// Handy to not force the user to declare this, i.e. to not leak implementation details.
template <class Base>
struct WithDefaults : public Base, public StreamImplDefault {};

template <class T>
constexpr void assert_stream_holds_tuple() {
  static_assert(is_tuple_v<MessageOf<T>>, "The Stream must hold a tuple as a value for unpacking.");
}

// Assert that all Streams types hold the same value
template <AnyStream First, AnyStream... Rest>
constexpr void assert_all_stream_values_are_same() {
  // Static assert that each T::Value is the same as First::Value
  static_assert((std::is_same_v<MessageOf<First>, MessageOf<Rest>> && ...),
                "The values of all the streams must be the same");
}

template <class F, class Arg>
struct check_callback {
  static_assert(std::is_invocable_v<F, Arg>, "The callback has the wrong signature");
  using ReturnType = std::invoke_result_t<F, Arg>;
};

template <class F, class... Args>
struct check_callback<F, std::tuple<Args...>> {
  static_assert(std::is_invocable_v<F, Args...>,
                "The callback has the wrong signature, it has to take multiple arguments (the "
                "Stream holds tuple, the arguments are all the elements of the tuple)");
  using ReturnType = std::invoke_result_t<F, Args...>;
};

/// An awaiter required to implement the operator `co_await` for Streams. It it needed for
/// supporting C++ coroutines.
template <class S>
struct Awaiter {
  S &stream;
  Awaiter(S &s) : stream(s) {}
  /// @return Returns whether this Stream already has a value.
  bool await_ready() const noexcept {
    if (icey_coro_debug_print)
      std::cout << "Await ready on Stream " << get_type(stream) << " called" << std::endl;
    return !stream.impl()->has_none();
  }
  /// @brief Registers the continuation (that's the code that follows the `co_await` statement, in
  /// form of a function pointer) as a callback of the stream. This callback then get's called by
  /// the ROS executor.
  void await_suspend(std::coroutine_handle<> continuation) noexcept {
    if (icey_coro_debug_print)
      std::cout << "Await suspend on Stream " << get_type(stream) << " called" << std::endl;
    if (!stream.impl()->registered_continuation_callback_) {
      stream.impl()->register_handler([impl = stream.impl()](auto &) {
        /// Important: Call the continuation only once since we may be awaiting the stream only
        /// once
        if (impl->continuation_) {
          auto c = std::exchange(impl->continuation_,
                                 nullptr);  /// Get the continuation and replace it with nullptr
                                            /// so that it is called only once
          c.resume();
        }
      });
      stream.impl()->registered_continuation_callback_ = true;
    }
    stream.impl()->continuation_ = continuation;
  }
  /// @brief Returns the current value of the stream. If an exception occurred (but was not handled)
  /// previously, here it is re-thrown
  auto await_resume() const {
    if (icey_coro_debug_print)
      std::cout << "Await resume on Stream " << get_type(stream) << " called" << std::endl;
    if (stream.exception_ptr_)  /// [Coroutine support] The coroutines are specified so that the
                                /// compiler does not do exception handling
      /// everywhere, so they put the burden on the implementers to defer throwing the exception
      std::rethrow_exception(stream.exception_ptr_);
    return stream.impl()->take();
  }
};

/// \brief A stream, an abstraction over an asynchronous sequence of values.
/// It has a state of type Result and a list of callbacks that get notified when this state changes.
/// It is conceptually very similar to a promise in JavaScript except that state transitions are not
/// final. This is the base class for all the other streams.
///
/// \tparam _Value the type of the value
/// \tparam _Error the type of the error. It can also be an exception.
/// \tparam ImplBase a class from which the implementation (impl::Stream) derives, used as an
/// extention point.
/// \note This class does not have any fields except a weak pointer to the actual
/// implementation (i.e. it uses the PIMPL idiom). You should not add any fields when inheriting
/// form this class. Instead, put the additional fields that you need in a separate struct and pass
/// it as the `ImplBase` template parameter. These fields become available through
/// `impl().<my_field>`, (i.e. the Impl-class will derive from ImplBase).
template <class _Value, class _Error = Nothing, class ImplBase = Nothing>
class Stream : public StreamTag {
  static_assert(std::is_default_constructible_v<ImplBase>,
                "ImplBase must be default constructable");
  friend Context;
  friend Awaiter<Stream<_Value, _Error, ImplBase>>;

public:
  using Value = _Value;
  using Error = _Error;
  using Self = Stream<_Value, _Error, ImplBase>;
  /// The actual implementation of the Stream
  using Impl = impl::Stream<Value, Error, WithDefaults<ImplBase>, WithDefaults<Nothing>>;
  /// [Coroutine support]
  using promise_type = Self;

#ifdef ICEY_DEBUG_PRINT_STREAM_ALLOCATIONS
  Stream() { std::cout << "Created new Stream: " << get_type(*this) << std::endl; }
  ~Stream() { std::cout << "Destroying Stream: " << get_type(*this) << std::endl; }
#else
  Stream() = default;
#endif

  /// Create s new stream using the context.
  explicit Stream(Context &ctx) : impl_(ctx.create_stream_impl<Impl>()) {}
  explicit Stream(std::shared_ptr<Impl> impl) : impl_(impl) {}

  /// [Coroutine support]
  Self &get_return_object() { return *this; }
  /// [Coroutine support]
  std::suspend_never initial_suspend() { return {}; }
  /// [Coroutine support]
  std::suspend_never final_suspend() const noexcept { return {}; }
  /// [Coroutine support] return_value returns the value of the Steam.
  auto return_value() { return this->value(); }
  /// [Coroutine support] Store the unhandled exception in case it occurs: We will re-throw it when
  /// it's time. (The compiler can't do this for us because of reasons)
  void unhandled_exception() { exception_ptr_ = std::current_exception(); }

  /// [Coroutine support] Allow this stream to be awaited with `co_await stream` in C++20
  /// coroutines.
  Awaiter<Self> operator co_await() { return Awaiter{*this}; }

  /// [Coroutine support] Implementation of the operator co_return(x): It *sets* the value of the
  /// Stream object that is about to get returned (the compiler creates it beforehand)
  void return_value(const Value &x) {
    if (icey_coro_debug_print)
      std::cout << get_type(*this) << " setting value using operator co_return for "
                << boost::typeindex::type_id_runtime(x).pretty_name() << " called " << std::endl;
    this->impl()->set_value(x);
  }

  /// Returns a weak pointer to the implementation.
  Weak<Impl> impl() const { return impl_; }

  /// \brief Calls the given function (synchronous or asynchronous) f every time this stream
  /// receives a value.  /// It returns a new stream that receives the values that this function f
  /// returns. The returned Stream also passes though the errors of this stream so that chaining
  /// `then`s with an `except` works. \returns A new Stream that changes it's value to y every time
  /// this stream receives a value x, where y = f(x). The type of the returned stream is:
  /// - Stream<Nothing, _Error> if F is (X) -> void
  /// - Stream<NewValue, NewError> if F is (X) -> Result<NewValue, NewError>
  /// - Stream<NewValue, _Error> if F is (X) -> std::optional<NewValue>
  /// - Stream<Y, _Error> otherwise
  /// \tparam F: Must be (X) -> Y, where X is:
  ///  - V_1, ..., V_n if Value is std::tuple<V_1, ..., V_n>
  ///  - Value otherwise
  template <class F>
  auto then(F &&f) {
    check_callback<F, Value>{};
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream cannot have values, so you cannot register then() on it.");
    return create_from_impl(impl()->then(std::forward<F>(f)));
  }

  /// \brief Calls the given function (synchronous or asynchronous) f every time this Stream
  /// receives an error. It returns a new Stream that receives the values that this function f
  /// returns. \returns A new Stream that changes it's value to y every time this stream receives an
  /// error x, where y = f(x). The type of the returned stream is:
  /// - Stream<Nothing, Nothing> if F is (X) -> void
  /// - Stream<NewValue, NewError> if F is (X) -> Result<NewValue, NewError>
  /// - Stream<NewValue, Nothing> if F is (X) -> std::optional<NewValue>
  /// - Stream<Y, Nothing> otherwise
  /// \tparam F: Must be (X) -> Y, where X is:
  ///  - V_1, ..., V_n if Value is std::tuple<V_1, ..., V_n>
  ///  - Value otherwise
  template <class F>
  auto except(F &&f) {
    check_callback<F, Error>{};
    static_assert(!std::is_same_v<Error, Nothing>,
                  "This stream cannot have errors, so you cannot register except() on it.");
    return create_from_impl(impl()->except(std::forward<F>(f)));
  }

  /// Connect this Stream to the given output stream so that the output stream receives all the
  /// values.
  /// \todo remove this, use unwrap
  template <AnyStream Output>
  void connect_values(Output output) {
    this->impl()->register_handler([output_impl = output.impl()](const auto &new_state) {
      if (new_state.has_value()) {
        output_impl->put_value(new_state.value());
      } else if (new_state.has_error()) {
      }
    });
  }

  /*!
    \brief Creates a publisher so that every new value of this Stream will get published.
    \sa PublisherStream
  */
  void publish(const std::string &topic_name, const rclcpp::QoS &qos = rclcpp::SystemDefaultsQoS(),
               const rclcpp::PublisherOptions publisher_options = {}) {
    assert_we_have_context();
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, there is nothing to publish, so you cannot "
                  "call publish() on it.");
    this->template create_stream<PublisherStream<Value>>(topic_name, qos, publisher_options, this);
  }

  /*!
    \brief Creates a custom publisher so that every new value of this Stream will get published.
    \sa PublisherStream
  */
  template <AnyStream PublisherType, class... Args>
  void publish(Args &&...args) {
    assert_we_have_context();
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, there is nothing to publish, so you cannot "
                  "call publish() on it.");
    /// We create this through the context to register it for attachment to the ROS node
    auto output = this->template create_stream<PublisherType>(std::forward<Args>(args)...);
    this->connect_values(output);
  }

  /*!
    \brief Creates a transform publisher so that every new value of this Stream, which must be of
    type `geometry_msgs::msg::TransformStamped`, will get published. \sa TransformPublisherStream
  */
  void publish_transform() {
    assert_we_have_context();
    static_assert(std::is_same_v<Value, geometry_msgs::msg::TransformStamped>,
                  "The stream must hold a Value of type "
                  "geometry_msgs::msg::TransformStamped[::SharedPtr] to be able to call "
                  "publish_transform() on it.");
    this->template create_stream<TransformPublisherStream>(this);
  }

  /// Unpacks an Stream holding a tuple as value to multiple Streams for each tuple element.
  /// Given that `Value` is of type `std::tuple<Value1, Value2, ..., ValueN>`, it returns
  /// `std::tuple< Stream<Value1>, Stream<Value2>, ..., Stream<ValueN>>`
  auto unpack() {
    static_assert(!std::is_same_v<Value, Nothing>,
                  "This stream does not have a value, there is nothing to unpack().");
    static_assert(is_tuple_v<Value>, "The Value must be a tuple for .unpack()");
    constexpr size_t tuple_sz = std::tuple_size_v<ValueOf<Self>>;
    /// hana::to<> is needed to make a sequence from a range, otherwise we cannot transform it, see
    /// https://stackoverflow.com/a/33181700
    constexpr auto indices = hana::to<hana::tuple_tag>(hana::range_c<std::size_t, 0, tuple_sz>);
    auto hana_tuple_output = hana::transform(indices, [*this](auto I) mutable {
      return then([I](const auto &...args) {  /// Need to take variadic because then()
                                              /// automatically unpacks tuples
        return std::get<I>(std::forward_as_tuple(
            args...));  /// So we need to pack this again in a tuple and get the index.
      });
    });
    /// Now create a std::tuple from the hana::tuple to be able to use structured bindings
    return hana::unpack(hana_tuple_output,
                        [](const auto &...args) { return std::make_tuple(args...); });
  }

  /// Unwraps, i.e. creates an ErrorFreeStream by handling the error with the given function f.
  /// The returned Stream will receive only the values of this stream.
  /// \tparam F Function receiving the Error of this
  /// Stream (it is unpacked if it's a tuple) and returning void.
  template <class F>
  Stream<Value, Nothing> unwrap_or(F &&f) {
    /// TODO Can't we just move this stream in, i.e. reuse it instead of creating a new one ? (In
    /// this case moving means re-using the impl)
    auto output = this->template create_stream<Stream<Value, Nothing>>();
    this->impl()->register_handler(
        [output_impl = output.impl(), f = std::forward<F>(f)](const auto &new_state) {
          if (new_state.has_value()) {
            output_impl->put_value(new_state.value());
          } else if (new_state.has_error()) {
            impl::unpack_if_tuple(f, new_state.error());
          }
        });
    return output;
  }

  /// Outputs the Value only if the given predicate f returns true.
  /// \tparam F Function receiving the Value of this Stream (it is unpacked if it's a tuple) and
  /// returning bool
  template <class F>
  Stream<Value, Error> filter(F f) {
    return this->then([f = std::move(f)](auto x) -> std::optional<Value> {
      if (!f(x))
        return {};
      else
        return x;
    });
  }

  /// Buffers N elements. Each time N elements were accumulated, the returned Stream will yield an
  /// array of exactly N elements.
  Buffer<Value> buffer(std::size_t N) const {
    return this->template create_stream<Buffer<Value>>(N, *this);
  }

  /// \return A new Stream that errors on a timeout, i.e. when this stream has not received any
  /// value for some time `max_age`.
  /// \param max_age the maximum age a message is allowed to have before the timeout occurs
  /// \param create_extra_timer If set to false, the timeout will
  /// only be detected after at least one message was received. If set to true, an extra timer is
  /// created so that timeouts can be detected even if no message is received.
  TimeoutFilter<Value> timeout(const Duration &max_age, bool create_extra_timer = true) {
    assert_we_have_context();
    /// We create this through the context to register it for the ROS node
    return this->template create_stream<TimeoutFilter<Value>>(*this, max_age, create_extra_timer);
  }

  // clang-format off
  /*!
    \brief Synchronizes a topic with a transform using the `tf2_ros::MessageFilter`.
    \param target_frame the transform on which we wait is specified by source_frame and target_frame, where source_frame is the frame in the header of the message. 
    \param lookup_timeout The maximum time to wait until the transform gets available for a message
    
    Example:
    \verbatim
      /// Synchronize with a transform: This will yield the message and the transform from the child_frame_id of the header message
      /// and the given target_frame ("map") at the time of the header stamp. It will wait up to 200ms for the transform. 
      node->icey().create_subscription<sensor_msgs::msg::Image>("camera")
        .synchronize_with_transform("map", 200ms)
        .unwrap_or([&](std::string error) { RCLCPP_INFO_STREAM(node->get_logger(), "Transform lookup error: " << error);})
        .then([](sensor_msgs::msg::Image::SharedPtr image, 
            const geometry_msgs::msg::TransformStamped &transform_to_map) {

        });
    \endverbatim
  */
  // clang-format on
  TransformSynchronizer<Value> synchronize_with_transform(const std::string &target_frame,
                                                          const Duration &lookup_timeout) {
    assert_we_have_context();
    static_assert(!std::is_same_v<Value, Nothing>, "");
    /// TODO assert has header
    return this->template create_stream<TransformSynchronizer<Value>>(target_frame, lookup_timeout,
                                                                      this);
  }

  /// Creates a new stream of type S using the Context. See Context::create_stream
  template <AnyStream S, class... Args>
  S create_stream(Args &&...args) const {
    return this->impl()->context.lock()->template create_stream<S>(std::forward<Args>(args)...);
  }

protected:
  void assert_we_have_context() {
    if (!this->impl()->context.lock())
      throw std::runtime_error("This stream does not have context");
  }

  /// Pattern-maching factory function that creates a New Self with different value and error types
  /// based on the passed implementation pointer.
  /// (this is only needed for impl::Stream::done, which creates a new stream that always has
  /// Derived stripped off, i.e. set to Nothing.)
  /// We do NOT add the impl to the context since it is already captured by the handler.
  template <class NewVal, class NewErr>
  Stream<NewVal, NewErr> create_from_impl(
      const std::shared_ptr<
          impl::Stream<NewVal, NewErr, WithDefaults<Nothing>, WithDefaults<Nothing>>> &impl) const {
    Stream<NewVal, NewErr> new_stream(impl);
    new_stream.impl()->context = this->impl()->context;
    return new_stream;
  }

  /// The pointer to the undelying implementation (i.e. PIMPL idiom).
  Weak<Impl> impl_{nullptr};
  std::exception_ptr exception_ptr_{nullptr};
};

template <class Value>
struct BufferImpl {
  std::size_t N{1};
  std::shared_ptr<std::vector<Value>> buffer{std::make_shared<std::vector<Value>>()};
};

/// This stream is created when calling Stream::buffer. A Buffer is a Stream that holds an array of
/// values. It accumulates a certain amount of values and only then it has itself a value. It does
/// not have errors since it does not make much sense to accumulate errors. \sa Stream::buffer
template <class Value>
struct Buffer : public Stream<std::shared_ptr<std::vector<Value>>, Nothing, BufferImpl<Value>> {
  using Base = Stream<std::shared_ptr<std::vector<Value>>, Nothing, BufferImpl<Value>>;
  template <ErrorFreeStream Input>
  explicit Buffer(Context &context, std::size_t N, Input input) : Base(context) {
    this->impl()->N = N;
    input.impl()->register_handler([impl = this->impl()](auto x) {
      impl->buffer->push_back(x.value());
      if (impl->buffer->size() == impl->N) {
        impl->put_value(impl->buffer);
        impl->buffer->clear();
      }
    });
  }
};

/// What follows are parameters.
/// Traits to recognize valid types for ROS parameters (Reference:
/// https://docs.ros.org/en/jazzy/p/rcl_interfaces/interfaces/msg/ParameterValue.html)
template <class T>
struct is_valid_ros_param_type : std::false_type {};
template <>
struct is_valid_ros_param_type<bool> : std::true_type {};
template <>
struct is_valid_ros_param_type<int64_t> : std::true_type {};

template <>
struct is_valid_ros_param_type<double> : std::true_type {};
template <>
struct is_valid_ros_param_type<std::string> : std::true_type {};
/// Array type
template <class T>
struct is_valid_ros_param_type<std::vector<T>> : is_valid_ros_param_type<T> {};
template <class T, std::size_t N>
struct is_valid_ros_param_type<std::array<T, N>> : is_valid_ros_param_type<T> {};
/// Byte array, extra specialization since byte is not allowed as scalar type, only byte arrays
template <>
struct is_valid_ros_param_type<std::vector<std::byte>> : std::true_type {};
template <std::size_t N>
struct is_valid_ros_param_type<std::array<std::byte, N>> : std::true_type {};

template <class T>
struct t_is_std_array : std::false_type {};

template <class T, std::size_t N>
struct t_is_std_array<std::array<T, N>> : std::true_type {};

template <class T>
constexpr bool is_std_array = t_is_std_array<T>::value;

/// What follows, is an improved parameters API. For API docs, see
/// https://docs.ros.org/en/jazzy/p/rcl_interfaces First, some constraints we can impose on
/// parameters:

/// A closed interval, meaning a value must be greater or equal to a minimum value
/// and less or equal to a maximum value.
template <class Value>
struct Interval {
  static_assert(std::is_arithmetic_v<Value>, "The value type must be a number");
  // Interval(Like<Value> auto _minimum, Like<Value> auto _maximum): minimum(_minimum),
  // maximum(_maximum) {}
  Interval(Value _minimum, Value _maximum) : minimum(_minimum), maximum(_maximum) {}
  Value minimum;
  Value maximum;
};

/// A set specified by a given list of values that are in the set.
template <class Value>
struct Set {
  Set(std::vector<Value> l) : set_of_values(l.begin(), l.end()) {}
  // explicit Set(const Value ...values) : set_of_values(values...) {}
  std::unordered_set<Value> set_of_values;
};

/*!
   A parameter validator, validating parameter updates. It is able to constrain parameter values.
   It is essentially a function that returns true if a value is allowed and false otherwise.
   \tparam Value the parameter type (i.e. double)
*/
template <class Value>
struct Validator {
  using ROSValue = std::conditional_t<std::is_unsigned_v<Value>, int, Value>;
  /// The type of the validator predicate, meaning the function that returns an error if the
  /// parameter update is rejected and an empty string otherwise. Why we don't return a
  /// std::optional<std::string> ?
  ///  Because this way we can ensure through the type system that if a parameter update is
  ///  rejected, it is always rejected for a reason.
  using Validate = std::function<std::string(const rclcpp::Parameter &)>;

  /// Allow default-constructed validator, by default allowing all values.
  Validator() {
    if constexpr (std::is_unsigned_v<Value>) {
      descriptor.integer_range.resize(1);
      descriptor.integer_range.front().from_value = 0;
      descriptor.integer_range.front().to_value = std::numeric_limits<int64_t>::max();
      descriptor.integer_range.front().step = 1;
      /// When an interval is set with the descriptor, ROS already validates the parameters, our
      /// custom validator won't even get called.
      validate = get_default_validator();
    } else {
      /// ROS already validates type changes (and disallows them).
      validate = get_default_validator();
    }
  }

  /// Construct explicitly from a validation function (predicate).
  explicit Validator(const Validate &validate) : validate(validate) {}

  /// Allow implicit conversion from some easy sets:
  Validator(const Interval<Value> &interval)  // NOLINT
  {
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range.front().from_value = interval.minimum;
    descriptor.floating_point_range.front().to_value = interval.maximum;
    descriptor.floating_point_range.front().step = 0.1;
    /// When an interval is set with the descriptor, ROS already validates the parameters, our
    /// custom validator won't even be called.
    validate = get_default_validator();
  }

  /// Implicit conversion from a Set of values
  Validator(const Set<Value> &set)  // NOLINT
  {
    validate = [set](const rclcpp::Parameter &new_param) {
      auto new_value = new_param.get_value<ROSValue>();
      if (!set.set_of_values.count(new_value))
        return "The given value is not in the set of allowed values";
      return "";
    };
  }

  Validate get_default_validator() const {
    return [](const rclcpp::Parameter &) { return ""; };
  }

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  /// A predicate that indicates whether a given value is feasible.
  /// By default, a validator always returns true since the parameter is unconstrained
  Validate validate;
};

/// An stream representing parameters. It always registers the callback to obtain the parameter
/// updates.
template <class _Value>
struct ParameterStream : public Stream<_Value> {
  using Base = Stream<_Value>;
  using Value = _Value;
  static_assert(is_valid_ros_param_type<_Value>::value,
                "Type is not an allowed ROS parameter type");
  ParameterStream() = default;
  /// @brief A constructor that should only be used for parameter structs. It does not set the name
  /// of the parameter and therefore leaves this ParameterStream in a not fully initialized state.
  /// Context::declare_parameter_struct later infers the name of this parameter from the name of the
  /// field in the parameter struct and sets it before registering the parameter with ROS.
  /// @param default_value
  /// @param validator the validator implementing constraints
  /// @param description the description written in the ParameterDescriptor
  /// @param read_only if yes, the parameter cannot be modified
  /// @param ignore_override
  ParameterStream(const Value &default_value, const Validator<Value> &validator = {},
                  std::string description = "", bool read_only = false,
                  bool ignore_override = false) {
    this->default_value = default_value;
    this->validator = validator;
    this->description = description;
    this->read_only = read_only;
    this->ignore_override = ignore_override;
  }

  /// @brief The standard constructor used when declaring parameters with
  /// Context::declare_parameter.
  /// @param context the ICEY context
  /// @param parameter_name
  /// @param default_value
  /// @param validator the validator implementing constraints
  /// @param description
  /// @param read_only if set to true, parameter updates will be rejected
  /// @param ignore_override
  ParameterStream(Context &context, const std::string &parameter_name, const Value &default_value,
                  const Validator<Value> &validator = {}, std::string description = "",
                  bool read_only = false, bool ignore_override = false)
      : Base(context) {
    this->parameter_name = parameter_name;
    this->default_value = default_value;
    this->validator = validator;
    this->description = description;
    this->read_only = read_only;
    this->ignore_override = ignore_override;
    this->register_with_ros(context);
  }

  /// Register this paremeter with the ROS node, meaning it actually calls
  /// node->declare_parameter(). After calling this method, this ParameterStream will have a value.
  void register_with_ros(Context &context) {
    context.add_parameter<Value>(
        this->parameter_name, this->default_value,
        [impl = this->impl()](const rclcpp::Parameter &new_param) {
          impl->put_value(new_param.get_value<_Value>());
        },
        this->create_descriptor(), this->validator.validate, this->ignore_override);
    /// Set the default value
    this->impl()->put_value(context.node_base().get_parameter<Value>(this->parameter_name));
  }

  /// Get the value. Parameters are initialized always at the beginning, so they always have a
  /// value.
  const Value &value() const {
    if (!this->impl()->has_value()) {
      throw std::runtime_error("Parameter '" + this->parameter_name + "' does not have a value");
    }
    return this->impl()->value();
  }

  /// Allow implicit conversion to the stored value type for consistent API between constrained and
  /// non-constrained parameters when using the parameter structs.
  operator Value() const  // NOLINT
  {
    return this->value();
  }
  std::string parameter_name;

protected:
  auto create_descriptor() {
    auto desc = validator.descriptor;
    desc.name = parameter_name;
    desc.description = description;
    desc.read_only = read_only;
    return desc;
  }

  Value default_value;
  Validator<Value> validator;
  std::string description;
  bool read_only = false;
  bool ignore_override = false;
};

/// A class that abstracts a plain value and a ParameterStream so that both are supported in promise
/// mode. This is needed for example for timeouts and coordinate system names.
template <class Value>
struct ValueOrParameter {
  ValueOrParameter() = default;

  /// Construct a ValueOrParameter implicitly from a value
  ValueOrParameter(const Value &value)  // NOLINT
      : get([value]() { return value; }) {}

  /// Construct a ValueOrParameter implicitly from a something similar to a value, For example,
  /// `"hello"` is a const char[5] literal that is convertible to a std::string, the value.
  template <class T>
  requires std::convertible_to<T, Value> &&(!AnyStream<T>)ValueOrParameter(const T &v)  // NOLINT
      : get([value = Value(v)]() { return value; }) {}

  /// Construct a ValueOrParameter implicitly from parameter (i.e. ParameterStream)
  ValueOrParameter(const ParameterStream<Value> &param)  // NOLINT
      : get([param_impl = param.impl()]() { return param_impl.lock()->get_value(); }) {}

  /// We use a std::function for the type erasure: When called, it obtains the current value
  std::function<Value()> get;
};

template <class _Message>
struct SubscriptionStreamImpl {
  std::shared_ptr<rclcpp::Subscription<_Message>> subscription;
};
/// A stream that represents a regular ROS subscription. It stores as its value always a shared
/// pointer to the message.
template <class _Message>
struct SubscriptionStream
    : public Stream<typename _Message::SharedPtr, Nothing, SubscriptionStreamImpl<_Message>> {
  using Base = Stream<typename _Message::SharedPtr, Nothing, SubscriptionStreamImpl<_Message>>;
  using Value = typename _Message::SharedPtr;
  using SyncCallback = std::function<void(Value)>;
  using AsyncCallback = std::function<Promise<void>(Value)>;
  SubscriptionStream() = default;
  SubscriptionStream(Context &context, const std::string &topic_name, const rclcpp::QoS &qos,
                     const rclcpp::SubscriptionOptions &options)
      : Base(context) {
    this->impl()->subscription = context.node_base().create_subscription<_Message>(
        topic_name,
        [impl = this->impl()](typename _Message::SharedPtr msg) { impl->put_value(msg); }, qos,
        options);
  }
};

struct TransformSubscriptionStreamImpl {
  ValueOrParameter<std::string> source_frame;
  ValueOrParameter<std::string> target_frame;
  /// We do not own the listener, the Book owns it
  Weak<TFListener> tf2_listener;
};

/// A Stream that represents a subscription between two coordinate systems. (See TFListener)
/// It yields a value each time the transform between these two coordinate systems changes.
struct TransformSubscriptionStream : public Stream<geometry_msgs::msg::TransformStamped,
                                                   std::string, TransformSubscriptionStreamImpl> {
  using Base =
      Stream<geometry_msgs::msg::TransformStamped, std::string, TransformSubscriptionStreamImpl>;
  using Message = geometry_msgs::msg::TransformStamped;
  using Self = TransformSubscriptionStream;
  TransformSubscriptionStream() = default;
  TransformSubscriptionStream(Context &context, ValueOrParameter<std::string> target_frame,
                              ValueOrParameter<std::string> source_frame)
      : Base(context) {
    this->impl()->target_frame = target_frame;
    this->impl()->source_frame = source_frame;
    this->impl()->tf2_listener = context.add_tf_subscription(
        target_frame.get, source_frame.get,
        [impl = this->impl()](const geometry_msgs::msg::TransformStamped &new_value) {
          impl->put_value(new_value);
        },
        [impl = this->impl()](const tf2::TransformException &ex) { impl->put_error(ex.what()); });
  }
};

struct TimerImpl {
  size_t ticks_counter{0};
  rclcpp::TimerBase::SharedPtr timer;
};

/// A Stream representing a ROS-Timer. It saves the number of ticks as it's value.
/// TODO use this TimerInfo thingy, it actually exists in ROS 2 as well.
struct TimerStream : public Stream<size_t, Nothing, TimerImpl> {
  using Base = Stream<size_t, Nothing, TimerImpl>;
  TimerStream() = default;
  TimerStream(Context &context, const Duration &interval, bool is_one_off_timer) : Base(context) {
    this->impl()->timer =
        context.node_base().create_wall_timer(interval, [impl = this->impl(), is_one_off_timer]() {
          /// Needed as separate state as it might be resetted in async/await mode
          auto cnt = impl->ticks_counter;
          impl->ticks_counter++;
          if (is_one_off_timer) impl->timer->cancel();
          impl->put_value(cnt);
        });
  }
  void reset() { this->impl()->timer->reset(); }
  void cancel() { this->impl()->timer->cancel(); }
};

template <class _Value>
struct PublisherImpl {
  std::shared_ptr<rclcpp::Publisher<remove_shared_ptr_t<_Value>>> publisher;
  void publish(const _Value &message) {
    /// TODO(Ivo) We always copy the message for publishing because we do not support so-called
    /// loaned messages: https://design.ros2.org/articles/zero_copy.html The following comment
    /// explains the reasoning with technical/API issues, but conceptually we simply do not support
    /// loaned messages.
    // Comment: We cannot pass over the pointer since publish expects a unique ptr and we got a
    // shared ptr. We cannot just promote the unique ptr to a shared ptr because we cannot ensure
    // the shared ptr is not referenced somewhere else.
    /// We could check whether use_count is one but this is not a reliable indicator whether the
    /// object not referenced anywhere else.
    // This is because the use_count can change in a multithreaded program immediately after it was
    // retrieved (i.e. a race occurs), see:
    /// https://en.cppreference.com/w/cpp/memory/shared_ptr/use_count (Same holds for
    /// shared_ptr::unique, which is defined simply as shared_ptr::unique -> bool: use_count() == 1)
    /// Therefore, we have to copy the message for publishing.
    if constexpr (is_shared_ptr<_Value>)
      publisher->publish(*message);
    else
      publisher->publish(message);
  }
};

/// A Stream representing a ROS-publisher.
/// \tparam _Value Can be either a `Message` or `std::shared_ptr<Message>` where `Message` must be
/// something publishable, i.e. either a valid ROS message or a masqueraded type.
template <class _Value>
struct PublisherStream : public Stream<_Value, Nothing, PublisherImpl<_Value>> {
  using Base = Stream<_Value, Nothing, PublisherImpl<_Value>>;
  using Message = remove_shared_ptr_t<_Value>;
  PublisherStream() = default;
  template <AnyStream Input = Stream<_Value>>
  PublisherStream(Context &context, const std::string &topic_name,
                  const rclcpp::QoS qos = rclcpp::SystemDefaultsQoS(),
                  const rclcpp::PublisherOptions publisher_options = {},
                  Input *maybe_input = nullptr)
      : Base(context) {
    this->impl()->publisher =
        context.node_base().create_publisher<Message>(topic_name, qos, publisher_options);
    this->impl()->register_handler(
        [impl = this->impl()](const auto &new_state) { impl->publish(new_state.value()); });
    if (maybe_input) {
      maybe_input->connect_values(*this);
    }
  }
  void publish(const _Value &message) const { this->impl()->publish(message); }
};

// A Stream representing a transform broadcaster that publishes transforms on TF.
struct TransformPublisherStreamImpl {
  Weak<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

struct TransformPublisherStream
    : public Stream<geometry_msgs::msg::TransformStamped, Nothing, TransformPublisherStreamImpl> {
  using Base = Stream<geometry_msgs::msg::TransformStamped, Nothing, TransformPublisherStreamImpl>;
  using Value = geometry_msgs::msg::TransformStamped;
  TransformPublisherStream() = default;
  template <AnyStream Input = Stream<geometry_msgs::msg::TransformStamped>>
  TransformPublisherStream(Context &context, Input *input = nullptr) : Base(context) {
    this->impl()->tf_broadcaster = context.add_tf_broadcaster_if_needed();
    this->impl()->register_handler([impl = this->impl()](const auto &new_state) {
      impl->tf_broadcaster->sendTransform(new_state.value());
    });
    if (input) input->connect_values(*this);
  }

  /// Publish a single message
  void publish(const geometry_msgs::msg::TransformStamped &message) {
    this->impl()->tf_broadcaster->sendTransform(message);
  }
};

/// A filter that detects timeouts, i.e. whether a value was received in a given time window.
/// It simply passes over the value if no timeout occurred, and errors otherwise.
/// \tparam _Value the value must be a message that has a header stamp
template <class Value>
struct TimeoutFilter
    : public Stream<Value, std::tuple<rclcpp::Time, rclcpp::Time, rclcpp::Duration>> {
  using Base = Stream<Value, std::tuple<rclcpp::Time, rclcpp::Time, rclcpp::Duration>>;
  /// Construct the filter an connect it to the input.
  /// \tparam Input another Stream that holds as a value a ROS message with a header stamp
  /// \param node the node is needed to know the current time
  /// \param input another Stream which is the input to this filter
  /// \param max_age a maximum age the message is allowed to have.
  /// \param create_extra_timer If set to false, the timeout will only be detected after at least
  /// one message was received. If set to true, an extra timer is created so that timeouts can be
  /// detected even if no message is received
  template <AnyStream Input>
  TimeoutFilter(Context &context, Input input, const Duration &max_age,
                bool create_extra_timer = true)
      : Base(context) {
    auto node_clock = context.node_base().get_node_clock_interface();
    rclcpp::Duration max_age_ros(max_age);
    auto check_state = [impl = this->impl(), node_clock, max_age_ros](const auto &new_state) {
      if (!new_state.has_value()) return true;
      const auto &message = new_state.value();
      rclcpp::Time time_now{node_clock->get_clock()->now()};
      rclcpp::Time time_message{message->header.stamp};
      if ((time_now - time_message) <= max_age_ros) {
        impl->put_value(message);
        return false;
      } else {
        impl->put_error(std::make_tuple(time_now, time_message, max_age_ros));
        return true;
      }
    };
    if (create_extra_timer) {
      auto timer = input.impl()->context.lock()->create_timer(max_age);
      timer.then([timer_impl = timer.impl(), input_impl = input.impl(), check_state](size_t) {
        bool timeout_occurred = check_state(input_impl->get_state());
        if (!timeout_occurred) timer_impl->timer->reset();
      });
    } else {
      input.impl()->register_handler(check_state);
    }
  }
};

/// This impl is needed because it makes the signalMessage method public, it is otherwise protected.
template <class Message>
struct SimpleFilterAdapterImpl : public message_filters::SimpleFilter<Message> {
  void signalMessage(auto event) { message_filters::SimpleFilter<Message>::signalMessage(event); }
};

/// Adapts the `message_filters::SimpleFilter` to our
/// `Stream` (which is a similar concept).
/// \note This is essentially the same as what
/// `message_filters::Subscriber` does:
/// https://github.com/ros2/message_filters/blob/humble/include/message_filters/subscriber.h#L349
template <class Message>
struct SimpleFilterAdapter
    : public Stream<typename Message::SharedPtr, Nothing, SimpleFilterAdapterImpl<Message>> {
  using Base = Stream<typename Message::SharedPtr, Nothing, SimpleFilterAdapterImpl<Message>>;
  /// Constructs a new instance and connects this Stream to the `message_filters::SimpleFilter` so
  /// that `signalMessage` is called every time this `icey::Stream` receives a value.
  SimpleFilterAdapter(Context &context) : Base(context) {
    this->impl()->register_handler([impl = this->impl()](const auto &new_state) {
      using Event = message_filters::MessageEvent<const Message>;
      if (new_state.has_value()) impl->signalMessage(Event(new_state.value()));
    });
  }
};

/// Holds a message_filters::Synchronizer and operates on it.
template <class... Messages>
struct ApproxTimeSynchronizerImpl {
  using Self = ApproxTimeSynchronizerImpl<Messages...>;
  using Policy = message_filters::sync_policies::ApproximateTime<Messages...>;
  using Synchronizer = message_filters::Synchronizer<Policy>;
  using Inputs = std::tuple<SimpleFilterAdapter<Messages>...>;
  const Inputs &inputs() const { return *inputs_; }

  /// TODO Hack we know that his class is derived from Derived, we will need to downcast
  /// to put the value in the Stream impl.
  /// A non-hack would be to change every impl to derive from impl::Stream
  /// We have to do this because we cannot capture this in the actual Stream but due to the
  /// very old (pre C++11) callback registering code in the message_filters package
  /// we cannot register a lambda where we could capture the impl. And at Humble, the variadic fix
  /// was not backported yet, as the variadic fix came only 2024.
  ///
  /// Note that this is like CRTP but but not done explicitly.
  /// So this is equally UB as CRTP is UB, not more, not less.
  /// See Stream::Impl
  using Derived = impl::Stream<std::tuple<typename Messages::SharedPtr...>, std::string,
                               WithDefaults<Self>, WithDefaults<Nothing>>;

  void create_synchronizer(Context &context, uint32_t queue_size) {
    queue_size_ = queue_size;
    inputs_ = std::make_shared<Inputs>(std::make_tuple(SimpleFilterAdapter<Messages>(context)...));
    auto synchronizer = std::make_shared<Synchronizer>(Policy(queue_size_));
    synchronizer_ = synchronizer;
    /// Connect with the input streams
    std::apply(
        [synchronizer](auto &...input_filters) {
          synchronizer->connectInput(*input_filters.impl()...);
        },
        *inputs_);
    /// This parameter setting is from the examples
    synchronizer_->setAgePenalty(0.50);
    synchronizer_->registerCallback(&Self::on_messages, this);
  }

  void on_messages(typename Messages::SharedPtr... msgs) {
    static_cast<Derived *>(this)->put_value(std::forward_as_tuple(msgs...));
  }

  uint32_t queue_size_{100};
  std::shared_ptr<Inputs> inputs_;
  std::shared_ptr<Synchronizer> synchronizer_;
};

/// A Stream representing an approximate time synchronizer from the message_filters package.
/// \sa synchronize_approx_time
template <class... Messages>
class ApproxTimeSynchronizer : public Stream<std::tuple<typename Messages::SharedPtr...>,
                                             std::string, ApproxTimeSynchronizerImpl<Messages...>> {
public:
  using Base = Stream<std::tuple<typename Messages::SharedPtr...>, std::string,
                      ApproxTimeSynchronizerImpl<Messages...>>;
  using Self = ApproxTimeSynchronizer<Messages...>;

  /// Constructs the synchronizer and connects it to the input streams so that it is ready to
  /// receive values.
  template <ErrorFreeStream... Inputs>
  ApproxTimeSynchronizer(Context &context, uint32_t queue_size, Inputs... inputs) : Base(context) {
    static_assert(sizeof...(Inputs) >= 2, "You need to synchronize at least two inputs.");
    this->impl()->create_synchronizer(context, queue_size);
    this->connect_inputs(inputs...);
  }

protected:
  /// Connects the given streams with the synchronizer inputs.
  template <ErrorFreeStream... Inputs>
  void connect_inputs(Inputs... inputs) {
    using namespace hana::literals;
    auto zipped = hana::zip(std::forward_as_tuple(inputs...), this->impl()->inputs());
    hana::for_each(zipped, [](auto &input_output_tuple) {
      auto &input_stream = input_output_tuple[0_c];
      auto &synchronizer_input = input_output_tuple[1_c];
      input_stream.connect_values(synchronizer_input);
    });
  }
};

/// Synchronizes a topic with a transform using tf2_ros::MessageFilter.
/// \sa Stream::synchronize_with_transform
template <class Message>
struct TransformSynchronizerImpl {
  using Self = TransformSynchronizerImpl<Message>;
  /// The book owns it.
  Weak<TFListener> tf_listener;
  std::string target_frame;
  std::shared_ptr<SimpleFilterAdapter<Message>> input_filter;
  std::shared_ptr<tf2_ros::MessageFilter<Message>> synchronizer;

  using Derived =
      impl::Stream<std::tuple<typename Message::SharedPtr, geometry_msgs::msg::TransformStamped>,
                   std::string, WithDefaults<Self>, WithDefaults<Nothing>>;

  void create_synchronizer(Context &context, const std::string &_target_frame,
                           const Duration &lookup_timeout) {
    tf_listener = context.add_tf_listener_if_needed();
    target_frame = _target_frame;
    input_filter = std::make_shared<SimpleFilterAdapter<Message>>(context);
    /// The argument "0" means here infinite message queue size. We set it so
    /// because we must buffer every message for as long as we are waiting for a transform.
    synchronizer = std::make_shared<tf2_ros::MessageFilter<Message>>(
        *input_filter->impl(), *tf_listener->buffer_, target_frame, 0,
        context.node_base().get_node_logging_interface(),
        context.node_base().get_node_clock_interface(), lookup_timeout);

    synchronizer->registerCallback(&Self::on_message, this);
  }

  void on_message(typename Message::SharedPtr message) {
    const auto timestamp = rclcpp_to_chrono(rclcpp::Time(message->header.stamp));
    auto maybe_transform =
        this->tf_listener->get_from_buffer(target_frame, message->header.frame_id, timestamp);
    if (maybe_transform.has_value())
      static_cast<Derived *>(this)->put_value(std::make_tuple(message, maybe_transform.value()));
    else  /// Invariant trap since I don't own the tf2_ros code and therefore can't prove it is
          /// correct:
      throw std::logic_error(
          "Invariant broke: tf2_ros::MessageFilter broke the promise that the transform is "
          "available");
  }
};

/// Synchronizes a topic with a transform using tf2_ros::MessageFilter.
template <class Value>
struct TransformSynchronizer
    : public Stream<std::tuple<Value, geometry_msgs::msg::TransformStamped>, std::string,
                    TransformSynchronizerImpl<remove_shared_ptr_t<Value>>> {
  using Base = Stream<std::tuple<Value, geometry_msgs::msg::TransformStamped>, std::string,
                      TransformSynchronizerImpl<remove_shared_ptr_t<Value>>>;
  /*!
    \brief Construct the TransformSynchronizer and connect it to the input.
    \param target_frame the transform on which we wait is specified by source_frame and
     target_frame, where source_frame is the frame in the header of the message
     \param lookup_timeout The maximum time to wait until the transform gets available for a message
  */
  template <ErrorFreeStream Input = Stream<int>>
  TransformSynchronizer(Context &context, const std::string &target_frame,
                        const Duration &lookup_timeout, Input *input = nullptr)
      : Base(context) {
    this->impl()->create_synchronizer(context, target_frame, lookup_timeout);
    if (input) {
      input->connect_values(*this->impl()->input_filter);
    }
  }
};

/*!
  Outputs the value or error of any of the inputs. All the inputs must have the same Value and
  Error type.
  \tparam Inputs A list of Stream<Value, Error> types, i.e. all the input Streams must have the same
  Value and Error type. \returns Stream<Value, Error>
*/
template <AnyStream... Inputs>
static auto any(Inputs... inputs) {
  // assert_all_stream_values_are_same<Inputs...>();
  auto first_input = std::get<0>(std::forward_as_tuple(inputs...));
  using Input = decltype(first_input);
  using InputValue = typename std::remove_reference_t<Input>::Value;
  using InputError = typename std::remove_reference_t<Input>::Error;
  /// First, create a new stream
  auto output = first_input.template create_stream<Stream<InputValue, InputError>>();
  /// Now connect each input with the output
  hana::for_each(std::forward_as_tuple(inputs...),
                 [output](auto &input) { input.connect_values(output); });
  return output;
}

/// Synchronize at least two streams by approximately matching the header time-stamps (using
/// the `message_filters::Synchronizer`). It accepts only an ErrorFreeStream since the synchronizer
/// may emit new errors. To obtain an ErrorFreeStream, use Stream::unwrap_or.
///
/// \tparam Inputs the input stream types, not necessarily all the same
/// \param queue_size the queue size to use, 100 is a good value.
/// \param inputs the input streams, not necessarily all of the same type
template <ErrorFreeStream... Inputs>
static ApproxTimeSynchronizer<MessageOf<Inputs>...> synchronize_approx_time(uint32_t queue_size,
                                                                            Inputs... inputs) {
  auto first_input = std::get<0>(std::forward_as_tuple(inputs...));  /// Use the first Stream
  return first_input.template create_stream<ApproxTimeSynchronizer<MessageOf<Inputs>...>>(
      queue_size, inputs...);
}

template <class ParameterT>
ParameterStream<ParameterT> Context::declare_parameter(const std::string &parameter_name,
                                                       const ParameterT &default_value,
                                                       const Validator<ParameterT> &validator,
                                                       std::string description, bool read_only,
                                                       bool ignore_override) {
  return create_stream<ParameterStream<ParameterT>>(parameter_name, default_value, validator,
                                                    description, read_only, ignore_override);
}

template <class ParameterStruct>
void Context::declare_parameter_struct(
    ParameterStruct &params, const std::function<void(const std::string &)> &notify_callback,
    std::string name_prefix) {
  field_reflection::for_each_field(params, [this, notify_callback, name_prefix](
                                               std::string_view field_name, auto &field_value) {
    using Field = std::remove_reference_t<decltype(field_value)>;
    std::string field_name_r = name_prefix + std::string(field_name);
    if constexpr (is_stream<Field>) {
      field_value.impl_ = this->create_stream_impl<typename Field::Impl>();
      field_value.impl()->context =
          this->shared_from_this();  /// First, give it the missing context
      field_value.parameter_name = field_name_r;
      if (notify_callback) {
        field_value.impl()->register_handler(
            [field_name_r, notify_callback](const auto &) { notify_callback(field_name_r); });
      }
      field_value.register_with_ros(*this);
    } else if constexpr (is_valid_ros_param_type<Field>::value) {
      this->declare_parameter<Field>(field_name_r, field_value)
          .impl()
          ->register_handler([&field_value, field_name_r, notify_callback](const auto &new_state) {
            field_value = new_state.value();
            if (notify_callback) {
              notify_callback(field_name_r);
            }
          });
    } else if constexpr (std::is_aggregate_v<Field>) {
      /// Else recurse for supporting grouped params
      declare_parameter_struct(field_value, notify_callback, field_name_r + ".");
    } else {
      /// static_assert(false) would always trigger, that is why we use this workaround, see
      /// https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2022/p2593r0.html
      static_assert(
          std::is_array_v<int>,
          "Every field of the parameters struct must be of type T or icey::Parameter<T> or "
          "a "
          "struct of such, where T is a valid ROS param type (see rcl_interfaces/ParameterType)");
    }
  });
}

template <class MessageT>
SubscriptionStream<MessageT> Context::create_subscription(
    const std::string &name, const rclcpp::QoS &qos, const rclcpp::SubscriptionOptions &options) {
  return create_stream<SubscriptionStream<MessageT>>(name, qos, options);
}

template <class MessageT, class Callback>
SubscriptionStream<MessageT> Context::create_subscription(
    const std::string &name, Callback &&cb, const rclcpp::QoS &qos,
    const rclcpp::SubscriptionOptions &options) {
  auto sub = create_stream<SubscriptionStream<MessageT>>(name, qos, options);
  sub.then(std::forward<Callback>(cb));
  return sub;
}

inline TransformSubscriptionStream Context::create_transform_subscription(
    ValueOrParameter<std::string> target_frame, ValueOrParameter<std::string> source_frame) {
  return create_stream<TransformSubscriptionStream>(target_frame, source_frame);
}

template <class Message>
PublisherStream<Message> Context::create_publisher(
    const std::string &topic_name, const rclcpp::QoS &qos,
    const rclcpp::PublisherOptions publisher_options) {
  return create_stream<PublisherStream<Message>>(topic_name, qos, publisher_options);
}

/// Create a publisher to publish transforms on the `/tf` topic. Works otherwise the same as
/// [tf2_ros::TransformBroadcaster].
inline TransformPublisherStream Context::create_transform_publisher() {
  return create_stream<TransformPublisherStream>();
}

/// The ROS node, additionally owning the context that holds the Streams.
/// \tparam NodeType can either be rclcpp::Node or rclcpp_lifecycle::LifecycleNode
template <class NodeType>
class NodeWithIceyContext : public NodeType {
public:
  /// Constructs a new new node and initializes the ICEY context.
  NodeWithIceyContext(std::string node_name,
                      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : NodeType(node_name, node_options) {
    /// Note that here we cannot call shared_from_this() since it requires the object to be already
    /// constructed.
    this->icey_context_ = std::make_shared<Context>(this);
  }

  /// Returns the ICEY context
  Context &icey() { return *this->icey_context_; }

protected:
  std::shared_ptr<Context> icey_context_;
};

/// The node type that you will use instead of an `rclcpp::Node`. It derives from the
/// `rclcpp::Node`, so that you can do everything that you can also with an `rclcpp::Node`. See
/// `NodeWithIceyContext` for details.
using Node = NodeWithIceyContext<rclcpp::Node>;
/// The node type that you will use instead of an `rclcpp_lifecycle::LifecycleNode`. It derives from
/// the `rclcpp_lifecycle::LifecycleNode`, so that you can do everything that you can also with an
/// `rclcpp_lifecycle::LifecycleNode`. See `NodeWithIceyContext` for details.
using LifecycleNode = NodeWithIceyContext<rclcpp_lifecycle::LifecycleNode>;

/// Parameter is a type to use only in parameter structs. Currently it is a Stream but this is
/// likely to change in the future
template <class Value>
using Parameter = ParameterStream<Value>;

}  // namespace icey
