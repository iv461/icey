#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>

/// TODO figure out where boost get's pulled in, we do not explicitly depend on it, but it's a
/// dependecy of one of our deps. It's not rclcpp and not message_filters.
#include <boost/hana.hpp>
#include <boost/hana/ext/std/tuple.hpp> /// Needed so that we do not need the custom hana tuples everywhere: https://stackoverflow.com/a/34318002
#include <boost/type_index.hpp>

#include <icey/impl/observable.hpp>
#include <icey/impl/bag_of_metaprogramming_tricks.hpp>

namespace hana = boost::hana;

namespace icey {

bool icey_debug_print = false;
using Time = ROS2Adapter::Time;
class Context;

/// Everything that can be "attached" to a ROS node, publishers, subscribers, clients, TF subscriber
/// etc.
class NodeAttachable  {
public:
  /// For creating new Observables, we need a reference to the context
  std::weak_ptr<Context> context;
  // The class name, i.e. the name of the type, for example "SubscriberObservable<std::string>"
  std::string class_name;
  /// A name to identify this node among multiple ones with the same type, usually the topic
  /// or service name
  std::string name;

  void attach_to_node(ROSAdapter::NodeHandle &node_handle) { 
    if (this->was_attached_) 
      throw std::invalid_argument("NodeAttachable was already attached");
    if(icey_debug_print)
      std::cout << "Attaching " << class_name << ", " << name << " ..." << std::endl;
    attach_(node_handle); 
    was_attached_ = true;
  }
  
  /// Needed to initialize parameters first
  bool is_parameter() const { return is_parameter_; }
  bool is_parameter_{false};
  bool was_attached_{false};
  std::function<void(ROSAdapter::NodeHandle &)> attach_;
};

struct ObservableTag{}; /// A tag to be able to recognize the type "Observeble" using traits
template <typename... Args>
struct observable_traits {
  static_assert(
      (is_shared_ptr<Args> && ...),
      "The arguments must be a shared_ptr< icey::Observable >, but it is not a shared_ptr");
  static_assert((std::is_base_of_v<ObservableTag, remove_shared_ptr_t<Args>> && ...),
                "The arguments must be an icey::Observable");
};

template <class T>
constexpr void assert_observable_holds_tuple() {
  static_assert(is_tuple_v<obs_msg<T>>, "The Observable must hold a tuple as a value for unpacking.");
}

// Assert that all Observables types hold the same value
template <typename First, typename... Rest>
constexpr void assert_all_observable_values_are_same() {
  observable_traits<First, Rest...>{};  /// Only Observables are expected to have ::Value
  // Static assert that each T::Value is the same as First::Value
  static_assert((std::is_same_v< obs_msg<First>, obs_msg<Rest>> && ...),
                "The values of all the observables must be the same");
}

/// An observable. Similar to a promise in JavaScript.
/// TODO Do not create shared_ptr, we finally have PIMPL
template <typename _Value, typename _ErrorValue = Nothing>
class Observable : public ObservableTag, public NodeAttachable,
    public std::enable_shared_from_this< Observable<_Value, _ErrorValue > >  {
  friend class Context;  /// To prevent misuse, only the Context is allowed to call set or
                         /// register_on_change_cb callbacks
public:
  using Impl = impl::Observable<_Value, _ErrorValue>;
  using Value = typename Impl::Value;
  using MaybeValue = typename Impl::MaybeValue;
  using ErrorValue =  typename Impl::ErrorValue;
  using Self = Observable<_Value, _ErrorValue>;

  void assert_we_have_context() {  //Cpp is great, but Java still has a NullPtrException more...
    if(!this->context.lock()) throw std::runtime_error("This observable does not have context, we cannot do stuff with it that depends on the context.");
  }

  /// Creates a new Observable that changes it's value to y every time the value x of the parent
  /// observable changes, where y = f(x).
  template <typename F>
  auto then(F &&f) {
    auto child = Self::create_from_impl(observable_->then(f));
    child->context = this->context;
    return child;
  }

  template <typename F>
  auto except(F &&f) { 
    static_assert(not std::is_same_v < ErrorValue, Nothing >, "This observable cannot have errors, so you cannot register .except() on it.");
    auto child = Self::create_from_impl(observable_->except(f));
    child->context = this->context;
    return child;
  }

  /// Create a ROS publisher by creating a new observable of type T and connecting it to this observable.
  template <class T, typename ...Args>
  void publish(Args &&... args) {
    assert_we_have_context();
    static_assert(not std::is_same_v < Value, Nothing >, "This observable does not have a value, there is nothing to publish, you cannot call publish() on it.");
    auto child = this->context.lock()->template create_observable<T>(args...);
    this->observable_->then([child](const auto &x) {child->observable_->resolve(x);});
  }

  /// Create a ROS normal publisher.
  void publish(const std::string &topic_name,
               const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQoS()) {
    assert_we_have_context();
    static_assert(not std::is_same_v < Value, Nothing >, "This observable does not have a value, there is nothing to publish, you cannot call publish() on it.");
    return this->context.lock()->create_publisher(this->shared_from_this(), topic_name, qos);
  }

  void publish_transform() {
    assert_we_have_context();
    static_assert(std::is_same_v < obs_msg<Self>, geometry_msgs::msg::TransformStamped >, "The observable must hold a Value of type geometry_msgs::msg::TransformStamped[::SharedPtr] to be able to call publish_transform() on it.");
    return this->context.lock()->create_transform_publisher(this->shared_from_this());
  }

//protected:
  /// Pattern-maching factory function that creates a New Self with different value and error types 
  /// based on the passed pointer to the implentation-Promise. Needed for then and except
  template<class NewVal, class NewErr>
  static std::shared_ptr < Observable<NewVal, NewErr> >
     create_from_impl(std::shared_ptr < impl::Observable<NewVal, NewErr> > obs_impl) {  
    auto new_obs = impl::create_observable<Observable<NewVal, NewErr>>();
    new_obs->observable_ = obs_impl;
    return new_obs;
  }
  std::shared_ptr<Impl> observable_{impl::create_observable<Impl>()};
};

/// An observable for ROS parameters. Fires initially an event if a default_value set
template <typename _Value>
class ParameterObservable : public Observable<_Value> {
public:
  using Value = _Value;
  using Base = Observable<_Value>;
  using MaybeValue = typename Base::MaybeValue;

  ParameterObservable(const std::string &parameter_name, const MaybeValue &default_value,
                      const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
                          rcl_interfaces::msg::ParameterDescriptor(),
                      bool ignore_override = false) {
    this->is_parameter_ = true;
    this->name = parameter_name;
    auto this_obs = this->observable_;
    this->attach_ = [=](ROSAdapter::NodeHandle &node) {
      node.declare_parameter<_Value>(
        parameter_name, default_value,
        [this_obs](const rclcpp::Parameter &new_param) {
          Value new_value = new_param.get_value<_Value>();
          this_obs->resolve(new_value);
        },parameter_descriptor, ignore_override);
      /// Set default value if there is one
      if (default_value) {
        Value initial_value;
        node.get_parameter_or(parameter_name, initial_value, *default_value);
        this_obs->resolve(initial_value);
      }
    };
  }

  /// Parameters are initialized always at the beginning, so we can provide getters for the value.
  const Value &value() const {
    if (!this->observable_->has_value()) {
      throw std::runtime_error(
          "Parameter '" + this->name +
          "' cannot be accessed before spawning the node. You can only access parameters "
          "inside callbacks (which are triggered after calling icey::spawn())");
    }
    return this->observable_->value();
  }
};

/// A subscriber observable, always stores a shared pointer to the message as it's value
template <typename _Message>
class SubscriptionObservable : public Observable < typename _Message::SharedPtr > {
  friend class Context;
public:
  using Value = typename _Message::SharedPtr;
  using Message = _Message;
  SubscriptionObservable(const std::string &topic_name, const ROSAdapter::QoS &qos,
                         const rclcpp::SubscriptionOptions &options) {
    this->name = topic_name;
    auto this_obs = this->observable_;  
    this->attach_ = [=](ROSAdapter::NodeHandle &node) {
      node.add_subscription<Value>(
          topic_name, [this_obs](typename Message::SharedPtr new_value) { this_obs->resolve(new_value); },
          qos, options);
    };
  }
};

/// An interpolatable observable is one that buffers the incoming messages using a circular buffer and
/// allows to query the message at a given point, using interpolation. 
struct InterpolateableObservableTag {};
template <typename _Message>
struct InterpolateableObservable : public InterpolateableObservableTag,
                                   public Observable < typename _Message::SharedPtr, std::string > {
  
  using MaybeValue = std::optional < typename _Message::SharedPtr >;
  /// Get the measurement at a given time point. Returns nothing if the buffer is empty or
  /// an extrapolation would be required.
  virtual MaybeValue get_at_time(const rclcpp::Time &time) const = 0;
};


template <typename T>
constexpr auto hana_is_interpolatable(T) {
    if constexpr(std::is_base_of_v<InterpolateableObservableTag, T>)
        return hana::bool_c<true>;
    else 
        return hana::bool_c<false>;    
}

/// A subscription for single transforms. It implements InterpolateableObservable but by using
/// lookupTransform, not an own buffer
struct TransformSubscriptionObservable
    : public InterpolateableObservable<geometry_msgs::msg::TransformStamped> {
public:
  using Message = geometry_msgs::msg::TransformStamped;
  using MaybeValue = InterpolateableObservable<Message>::MaybeValue;

  TransformSubscriptionObservable(const std::string &target_frame, const std::string &source_frame)
      : target_frame_(target_frame), source_frame_(source_frame) {
    this->name = "source_frame: " + source_frame_ + ", target_frame: " + target_frame_;
    this->attach_ = [=](ROSAdapter::NodeHandle &node) {
      auto this_obs = this->observable_;
      tf2_listener_ = node.add_tf_subscription(
          target_frame, source_frame,
          [this_obs](const geometry_msgs::msg::TransformStamped &new_value) {
          this_obs->resolve(std::make_shared<Message>(new_value)); /// TODO fix dynamic allocation on every call, instead allocate one object at the beginning in which we are going to write
          },
          [this_obs](const tf2::TransformException &ex) {
            this_obs->reject(ex.what());
          });
    };
  }
  MaybeValue get_at_time(const rclcpp::Time &time) const override {
    auto this_obs = this->observable_;
    try {
      // Note that this call does not wait, the transform must already have arrived. This works
      // because get_at_time() is called by the synchronizer
      auto tf_msg = tf2_listener_->buffer_.lookupTransform(target_frame_, source_frame_, time);
      /// TODO fix dynamic allocation on every call, instead allocate one object at the beginning in which we are going to write
      return std::make_shared<Message>(tf_msg); // For the sake of consistency, messages are always returned as shared pointers. Since lookupTransform gives us a value, we copy it over to a shared pointer.
    } catch (tf2::TransformException &e) {
      this_obs->reject(e.what());
      return {};
    }
  }
  std::string target_frame_;
  std::string source_frame_;
  std::shared_ptr<ROSAdapter::TFListener> tf2_listener_;
};

/// Timer signal, saves the number of ticks as the value and also passes the timerobject as well to
/// the callback
struct TimerObservable : public Observable<size_t> {
  using Value = size_t;
  TimerObservable(const ROSAdapter::Duration &interval, bool use_wall_time, bool is_one_off_timer) {
    this->name = "timer";
    auto this_obs = this->observable_;
    this->attach_ = [=](ROSAdapter::NodeHandle &node) {
      /// TODO DO NOT CAPTURE THIS, write the ros_timer_ somewhere
      timer = node.add_timer(interval, use_wall_time, [this, this_obs, is_one_off_timer]() {
        size_t ticks_counter = this_obs->has_value() ? this_obs->value() : 0;
        ticks_counter++;
        this_obs->resolve(ticks_counter);
        if (is_one_off_timer) 
          timer->cancel();
      });
    };
  }
  rclcpp::TimerBase::SharedPtr timer;
};

/// A publishabe state, read-only. Value can be either a Message or shared_ptr<Message> 
template <typename _Value>
class PublisherObservable : public Observable<_Value> {
  friend class Context;
public:
  using Value = _Value;
  using Base = Observable<_Value>;
  using Message = remove_shared_ptr_t<Value>; 
  static_assert(rclcpp::is_ros_compatible_type< Message >::value,
                "A publisher must use a publishable ROS message (no primitive types are possible)");
  PublisherObservable(const std::string &topic_name,
                      const ROSAdapter::QoS qos = ROS2Adapter::DefaultQoS()) {
    this->name = topic_name;
    auto this_obs = this->observable_;
    this->attach_ = [=](ROSAdapter::NodeHandle &node) {
      auto publisher = node.add_publication<Message>(topic_name, qos);
      this_obs->_register_handler([this_obs, publisher] () { 
          // We cannot pass over the pointer since publish expects a unique ptr and we got a shared_ptr. 
          // We cannot just create a unique_ptr because we cannot ensure we won't use the message even if use_count is one because use_count is meaningless in a multithreaded program. 
          const auto &new_value = this_obs->value(); /// There can be no error
          if constexpr(is_shared_ptr<Value>)
            publisher(*new_value);
          else 
            publisher(new_value);
      });
    };
  }
};

/// Wrap the message_filters official ROS package. In the following, "MFL" refers to the
/// message_filters package. An adapter, adapting the message_filters::SimpleFilter to our
/// Observable (two different implementations of almost the same concept). Does nothing else than
/// what message_filters::Subscriber does:
/// https://github.com/ros2/message_filters/blob/humble/include/message_filters/subscriber.h#L349
// We neeed the base to be able to recognize interpolatable nodes for example
template <typename _Message, class _Base = Observable< typename _Message::SharedPtr >>
struct SimpleFilterAdapter : public _Base, public message_filters::SimpleFilter<_Message> {
  SimpleFilterAdapter() {
    this->observable_->_register_handler([this]() {
      using Event = message_filters::MessageEvent<const _Message>;
      const auto &new_value = this->observable_->value(); /// There can be no error
      this->signalMessage(Event(new_value));
    });
  }
};

/// TODO this needs to check whether all inputs have the same QoS, so we will have do a walk
/// TODO adapt queue size automatically if we detect very different frequencies so that
/// synchronization still works. I would guess it works if the lowest frequency topic has a
/// frequency of at least 1/queue_size, if the highest frequency topic has a frequency of one.
template <typename... Messages>
class SynchronizerObservable : public Observable<std::tuple<typename Messages::SharedPtr...>, std::string> {
public:
  using Self = SynchronizerObservable<Messages...>;
  using Value = std::tuple<typename Messages::SharedPtr...>;
  /// Approx time will work as exact time if the stamps are exactly the same, so I wonder why the
  /// `TImeSynchronizer` uses by default ExactTime
  using Policy = message_filters::sync_policies::ApproximateTime<Messages...>;
  using Sync = message_filters::Synchronizer<Policy>;
  using Inputs = std::tuple<std::shared_ptr<SimpleFilterAdapter<Messages>>...>;
  SynchronizerObservable(uint32_t queue_size)
      : inputs_(std::make_shared<SimpleFilterAdapter<Messages>>()...),  /// They are dynamically
                                                                        /// allocated as is every
                                                                        /// other Observable
        queue_size_(queue_size) {
    this->create_mfl_synchronizer();
  }
  const auto &inputs() const { return inputs_; }
private:
  void on_messages(typename Messages::SharedPtr ...msgs) {
    this->observable_->resolve(std::forward_as_tuple(msgs...));
  }

  void create_mfl_synchronizer() {
    auto synchronizer = std::make_shared<Sync>(Policy(queue_size_));
    synchronizer_ = synchronizer;
    /// Connect with the input observables
    std::apply([synchronizer](auto &...input_filters) { synchronizer->connectInput(*input_filters...); },
               inputs_);
    synchronizer_->setAgePenalty(0.50);  /// TODO not sure why this is needed, present in example code
    auto this_obs = this->observable_; /// Important: Do not capture this
    synchronizer_->registerCallback(&Self::on_messages, this);  /// Register directly to impl::Observable
  }
  /// The input filters
  uint32_t queue_size_{10};
  Inputs inputs_;
  std::shared_ptr<Sync> synchronizer_;
};

// A transform broadcaster observable
class TransformPublisherObservable : public Observable<geometry_msgs::msg::TransformStamped> {
  friend class Context;
public:
  using Value = geometry_msgs::msg::TransformStamped;
  TransformPublisherObservable() {
      this->name = "tf_pub";
      auto this_obs = this->observable_;
      this->attach_ = [=](ROSAdapter::NodeHandle &node) {
        auto publish = node.add_tf_broadcaster_if_needed();
        this_obs->_register_handler([this_obs, publish]() { 
            const auto &new_value = this_obs->value(); /// There can be no error
            publish(new_value); 
        });
      };
  }
};

/// A service observable, storing it's request and response 
template <typename _ServiceT>
struct ServiceObservable
    : public Observable<std::pair<std::shared_ptr<typename _ServiceT::Request>,
                                  std::shared_ptr<typename _ServiceT::Response>>> {
  using Request = std::shared_ptr<typename _ServiceT::Request>;
  using Response = std::shared_ptr<typename _ServiceT::Response>;
  using Value = std::pair<Request, Response>;

  ServiceObservable(const std::string &service_name, const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
      auto this_obs = this->observable_;
      this->attach_ = [=](ROSAdapter::NodeHandle &node) {
        node.add_service<_ServiceT>(service_name,
            [this_obs](Request request, Response response) {
              this_obs->resolve(std::make_pair(request, response));
            }, qos);
      };
  }
};

/// A service client is a remote procedure call (RPC). It is a computation, and therefore an edge in the DFG
template <typename _ServiceT>
struct ServiceClient : public Observable <typename _ServiceT::Response::SharedPtr, std::string>{
  using Request = typename _ServiceT::Request::SharedPtr;
  using Response = typename _ServiceT::Response::SharedPtr;
  using Client = rclcpp::Client<_ServiceT>;
  ServiceClient(const std::string &service_name, const ROSAdapter::Duration &timeout): timeout_(timeout) {
      this->name = service_name; 
      auto this_obs = this->observable_;
      this->attach_ = [this, service_name](ROSAdapter::NodeHandle &node) {
        client_ = node.add_client<_ServiceT>(service_name);
      };
  }

  void call(Request request) const {
      using Future = typename Client::SharedFutureWithRequest;
      auto output_obs = this->observable_;
      if(!client_->wait_for_service(timeout_)) {
        output_obs->reject("SERVICE_UNAVAILABLE");
        return;
      }
      client_->async_send_request(
        request, [output_obs](Future response_futur) { 
          if(response_futur.valid()) {
            output_obs->resolve(response_futur.get().second);
          } else {
            /// TODO the FutureReturnCode enum has SUCCESS, INTERRUPTED, TIMEOUT as possible values. 
            /// I think since the async_send_request waits on the executor, we cannot interrupt it except with Ctrl-C
            if(!rclcpp::ok())
              output_obs->reject("rclcpp::FutureReturnCode::INTERRUPTED");
            else 
              output_obs->reject("rclcpp::FutureReturnCode::TIMEOUT");
            /// TODO Do the weird cleanup thing
          }
      });
  }
protected:
  typename Client::SharedPtr client_;  
  ROSAdapter::Duration timeout_;
};

/// Creates the data-flow graph and asserts that it is not edited one obtain() is called
/// A context, creates and manages the data-flow graph. Basis for the class-based API of ICEY.
struct Context : public std::enable_shared_from_this<Context> {
  virtual ~Context() {
    if (on_node_destruction_cb_) on_node_destruction_cb_();
  }

  /// Register callback to be called after all parameters have been attached
  /// For the functional API
  void register_after_parameter_initialization_cb(std::function<void()> cb) {
    after_parameter_initialization_cb_ = cb;
  }

  void register_on_node_destruction_cb(std::function<void()> cb) { on_node_destruction_cb_ = cb; }

  void initialize(ROSAdapter::NodeHandle &node) {
    if (attachables_.empty()) {
      std::cout << "WARNING: Nothing to spawn, try first to create some Observables"
                << std::endl;
      return;
    }
    attach_everything_to_node(node);
    was_initialized_ = true;
  }

  void attach_everything_to_node(ROSAdapter::NodeHandle &node) {
    /// Now attach everything to the ROS-Node, this creates the parameters, publishers etc.
    /// Now, allow for attaching additional nodes after we got the parameters. After attaching,
    /// parameters immediatelly have their values.
    if (icey_debug_print) std::cout << "[icey::Context] Attaching parameters ..." << std::endl;

    for (const auto &attachable: attachables_) {
      if (attachable->is_parameter()) attachable->attach_to_node(node);
    }
    if (icey_debug_print)
      std::cout << "[icey::Context] Attaching parameters finished." << std::endl;
  
    if (after_parameter_initialization_cb_)
      after_parameter_initialization_cb_(); 
    if (icey_debug_print) std::cout << "[icey::Context] Attaching maybe new vertices  ... " << std::endl;    
    for (const auto &attachable: attachables_) {
      if (!attachable->is_parameter()) attachable->attach_to_node(node);
    }
  }

  /// Creates a new observable of type O by passing the args to the constructor. It also adds it as
  /// a vertex to the graph.
  template <class O, typename... Args>
  std::shared_ptr<O> create_observable(Args &&...args) {
    assert_icey_was_not_initialized();
    auto observable = impl::create_observable<O>(args...);
    attachables_.push_back(observable);  /// Register with graph to obtain a vertex ID
    observable->context = shared_from_this();
    observable->class_name = boost::typeindex::type_id_runtime(*observable).pretty_name();
    return observable;
  }

  template <typename ParameterT>
  auto declare_parameter(const std::string &name,
                         const std::optional<ParameterT> &maybe_default_value = std::nullopt,
                         const rcl_interfaces::msg::ParameterDescriptor &parameter_descriptor =
                             rcl_interfaces::msg::ParameterDescriptor(),
                         bool ignore_override = false) {
    return create_observable<ParameterObservable<ParameterT>>(
        name, maybe_default_value, parameter_descriptor, ignore_override);
  }

  template <typename MessageT>
  auto create_subscription(
      const std::string &name, const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQoS(),
      const rclcpp::SubscriptionOptions &options = rclcpp::SubscriptionOptions()) {
    auto observable = create_observable<SubscriptionObservable<MessageT>>(name, qos, options);
    return observable;
  }

  auto create_transform_subscription(const std::string &target_frame,
                                     const std::string &source_frame) {
    return create_observable<TransformSubscriptionObservable>(target_frame, source_frame);
  }

  template <class Parent> 
  void create_publisher(Parent parent, const std::string &topic_name,
                        const ROS2Adapter::QoS &qos = ROS2Adapter::DefaultQoS()) {
    observable_traits<Parent>{};
    auto child = create_observable<PublisherObservable<obs_val<Parent>>>(topic_name, qos);
    parent->observable_->then([child](const auto &x) {child->observable_->resolve(x);});
  }

  template <class Parent>
  void create_transform_publisher(Parent parent) {
    observable_traits<Parent>{};
    auto child = create_observable<TransformPublisherObservable>();
    parent->observable_->then([child](const auto &x) {child->observable_->resolve(x);});
  }

  auto create_timer(const ROSAdapter::Duration &interval, bool use_wall_time = false,
                    bool is_one_off_timer = false) {
    return create_observable<TimerObservable>(interval, use_wall_time, is_one_off_timer);
  }

  template <typename ServiceT>
  auto create_service(const std::string &service_name,
                      const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    return create_observable<ServiceObservable<ServiceT>>(service_name, qos);
  }

  /// Add a service client
  template <typename ServiceT, typename Parent>
  auto create_client(Parent parent, const std::string &service_name,
                    const ROSAdapter::Duration &timeout,
                     const rclcpp::QoS &qos = rclcpp::ServicesQoS()) {
    observable_traits<Parent>{}; // obs_val since the Request must be a shared_ptr
    static_assert(std::is_same_v< obs_val<Parent>, typename ServiceT::Request::SharedPtr >, "The parent triggering the service must hold a value of type Request::SharedPtr");
    auto service_client = create_observable<ServiceClient<ServiceT>>(service_name, timeout);
    parent->then([service_client](auto req) { service_client->call(req); }); 
    return service_client;
  }

  /// Synchronizer that given a reference signal at its first argument, ouputs all the other topics
  // interpolated
  // TODO specialize when reference is a tuple of messages. In this case, we compute the arithmetic
  // TODO impl receive time. For this, this synchronizer must be an attachable because it needs to know the node's clock (that may be simulated time, i.e sub on /clock)
  template<class Reference, class... Parents> 
  static auto sync_with_reference(Reference reference, Parents ...parents) {
    observable_traits<Reference>{};
    observable_traits<Parents...>{};    
    using namespace message_filters::message_traits;
    static_assert(HasHeader<obs_msg<Reference>>::value, "The ROS message type must have a header with the timestamp to be synchronized");
    auto parents_tuple = std::make_tuple(parents...);
    /// TOOD somehow does not work
    //auto all_are_interpolatables = hana::all_of(parents_tuple,  [](auto t) { return hana_is_interpolatable(t); }); 
    //static_assert(all_are_interpolatables, "All inputs must be interpolatable when using the sync_with_reference");
    return reference->then([parents_tuple](const obs_val<Reference> &new_value) {
        auto parent_maybe_values = hana::transform(parents_tuple, [&](auto parent) {
              return parent->get_at_time(rclcpp::Time(new_value->header.stamp));
        });
        /// TODO If not hana::all(parent_maybe_values, have value) -> get error from parent and reject 
        return hana::prepend(parent_maybe_values, new_value);
    });
  }

  // TODO ID: [](auto &&x) { return std::move(x); }

  /// Synchronizer that synchronizes non-interpolatable signals by matching the time-stamps
  /// approximately
  template <typename... Parents>
  auto synchronize_approx_time(Parents... parents) {
    observable_traits<Parents...>{};
    static_assert(sizeof...(Parents), "You need to synchronize at least two inputs.");
    using namespace hana::literals;
    uint32_t queue_size = 10;
    auto synchronizer = create_observable<SynchronizerObservable< obs_msg<Parents>...>>(queue_size);
    auto zipped = hana::zip(std::forward_as_tuple(parents...), synchronizer->inputs());
    hana::for_each(zipped,
        [](auto &input_output_tuple) {
              auto &parent = input_output_tuple[0_c];
              auto &synchronizer_input = input_output_tuple[1_c];
              parent->then([synchronizer_input](const auto &x) { synchronizer_input->observable_->resolve(x); }); 
        });
    return synchronizer;
  }

  template <typename... Parents>
  auto synchronize(Parents... parents) {
    observable_traits<Parents...>{};
    static_assert(sizeof...(Parents) >= 2, "You need to have at least two inputs for synchronization.");
    auto parents_tuple = std::make_tuple(parents...);
    
    auto interpolatables = hana::remove_if(parents_tuple, [](auto t) { return not hana_is_interpolatable(t); });
    auto non_interpolatables = hana::remove_if(parents_tuple, [](auto t) { return hana_is_interpolatable(t); });
    constexpr int num_interpolatables = hana::length(interpolatables);
    constexpr int num_non_interpolatables = hana::length(non_interpolatables);
    static_assert(not (num_interpolatables > 0 && num_non_interpolatables == 0),
                  "You are trying to synchronize only interpolatable signals. This does not work, "
                  "you need to "
                  "have at least one non-interpolatable signal that is the common time for all the "
                  "interpolatable signals.");
    // This can only either be one or more than one. Precondition is that we synchronize at least
    // two entities. Given the condition above, the statement follows.
    if constexpr (num_non_interpolatables > 1) {
      /// We need the ApproxTime
      auto approx_time_output = hana::unpack(non_interpolatables, [this](auto ...parents) { return synchronize_approx_time(parents...);});
      if constexpr (num_interpolatables > 1) {
        /// We have interpolatables and non-interpolatables, so we need to use both synchronizers
        return hana::unpack(hana::prepend(interpolatables, approx_time_output), [this](auto ref, auto ...ints) { return sync_with_reference(ref, ints...); });
      } else {
        return approx_time_output;
      }
    } else {
      // Otherwise, we only need a sync with reference
      return hana::unpack(hana::prepend(interpolatables, std::get<0>(non_interpolatables)), 
          [](auto ref, auto ...ints) { return sync_with_reference(ref, ints...); });
    }
  }

  /// Serialize, pipe arbitrary number of parents of the same type into one. Needed for the
  /// control-flow where the same publisher can be called from multiple callbacks
  template <typename... Parents>
  auto serialize(Parents... parents) {
    observable_traits<Parents...>{};
    // TODO assert_all_observable_values_are_same<Parents...>();
    using Parent = decltype(*std::get<0>(std::forward_as_tuple(parents...)));
    using ParentValue = typename std::remove_reference_t<Parent>::Value;
    /// First, create a new observable
    auto child = create_observable<Observable<ParentValue>>();
    /// Now connect each parent with the child with the identity function
    hana::for_each(std::forward_as_tuple(parents...), 
        [child](auto &parent) {
              parent->then([child](const auto &x) { child->observable_->resolve(x); }); 
        });
    return child;
  }

  /// Now we can construct higher-order filters.
  /// For a tuple-observable, get it's N'th element
  template <int index, class Parent>
  auto get_nth(Parent &parent) {
    assert_observable_holds_tuple<Parent>();
    /// TODO add to graph, i.e. this->then(parent, f)
    return parent->then([](const auto &...args) {  /// Need to take variadic because then()
                                                   /// automatically unpacks tuples
      return std::get<index>(
          std::forward_as_tuple(args...));  /// So we need to pack this again in a tuple and get the index.
    });

  }

  /// Unpacks a observables of tuple into multiple observables
  /*
  template <class Parent>
  auto unpack(Parent &parent) {
  }
  */
  /// Then's on timeout. Creates a new timer.
  // timeout
  // Crates a new timer that sync_with_reference, matching exactly an output frequency. Input must be interpolatable.
  // throttle
  
  bool empty() const { return attachables_.empty(); }
  void clear() { attachables_.clear(); }

  void assert_icey_was_not_initialized() {
    if (was_initialized_)
      throw std::invalid_argument(
          "You are not allowed to add signals after ICEY was initialized. The graph must be "
          "static");
  }

  bool was_initialized_{false};
  std::vector< std::shared_ptr<NodeAttachable> > attachables_;  
  std::function<void()> after_parameter_initialization_cb_;
  std::function<void()> on_node_destruction_cb_;
};


/// The ROS node, additionally owning the data-flow graph (DFG) that contains the observables.
/// This class is needed to ensure that the context lives for as long as the node lives.
class ROSNodeWithDFG : public ROSAdapter::Node {
public:
  using Base = ROSAdapter::Node;
  using Base::Base;  // Take over all base class constructors

  /// This attaches all the ICEY signals to the node, meaning it creates the subcribes etc. It
  /// initializes everything in a pre-defined order.
  void icey_initialize() { icey().initialize(*this); }
  /// Returns the context that is needed to create ICEY observables
  Context &icey() { return *this->icey_context_; }
  std::shared_ptr<Context> icey_context_{std::make_shared<Context>()};
};

/// Blocking spawn of an existing node.
void spawn(std::shared_ptr<ROSAdapter::Node> node) {
  /// We use single-threaded executor because the MT one can starve due to a bug
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}

void spin_nodes(const std::vector<std::shared_ptr<ROSNodeWithDFG>> &nodes) {
  rclcpp::executors::SingleThreadedExecutor executor;
  /// This is how nodes should be composed according to ROS maintainer wjwwood:
  /// https://robotics.stackexchange.com/a/89767. He references
  /// https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp
  for (const auto &node : nodes) executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}

/// Public API aliases:
using Node = ROSNodeWithDFG;
template<class T>
using Parameter = ParameterObservable<T>;
}  // namespace icey

#include <icey/functional_api.hpp>
#define ICEY_ROS2_WAS_INCLUDED
