/// This header provides support to define a set of ROS parameters
/// including constraints and get updates if of them changes.
/// This greatly simplifies the declaration of many parameters
/// and comes close to the dynamic_reconfigure package in ROS 1.
/// The struct is declared at compile time and uses the field-reflection library to do static
/// reflection in C++20. We need the names of the fields and be able to access them by name (i.e.
/// parameters.my_param) while knowing their types. We also need the names of the fields as
/// runtime-strings. Note that in C++17 this is impossible to do without macros, only in C++20 it
/// became possible.
#pragma once

#include <functional>
#include <icey/impl/field_reflection.hpp>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace icey {

// Traits to recognize valid types for ROS parameters (Reference:
// https://docs.ros.org/en/jazzy/p/rcl_interfaces/interfaces/msg/ParameterValue.html)
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
struct is_valid_ros_param_type<std::vector<T> > : is_valid_ros_param_type<T> {};
template <class T, std::size_t N>
struct is_valid_ros_param_type<std::array<T, N> > : is_valid_ros_param_type<T> {};
/// Byte array, extra specialization since byte is not allowed as scalar type, only byte arrays
template <>
struct is_valid_ros_param_type<std::vector<std::byte> > : std::true_type {};
template <std::size_t N>
struct is_valid_ros_param_type<std::array<std::byte, N> > : std::true_type {};

/// Converters
/*template <class T>
struct to_ros_param_type {};

template <class T, std::size_t N>
struct to_ros_param_type< std::array<T, N> > { using type = std::vector<T>; };
*/

template <class T>
decltype(auto) to_ros_param_type(T &x) {
  if constexpr(is_std_array<T>) {
    return std::vector<typename T::value_type>(x.begin(), x.end());
  } else {
    return x;
  }
}

/// FOr API docs, see https://docs.ros.org/en/jazzy/p/rcl_interfaces

/// First, some constraints we can impose on parameters:
/// A closed interval, meaning a value must be greater or equal to a minimum value
/// and less or equal to a maximal value.
template <class Value>
struct Interval {
  static_assert(std::is_arithmetic_v<Value>, "The value type must be a number");
  /// TODO make two template params and use common_type + CTAD to make a really nice UX
  Interval(Value minimum, Value maximum) : minimum(minimum), maximum(maximum) {}
  Value minimum;
  Value maximum;
};

/// A set specified by a given list of values that are in the set.
template <class Value>
struct Set {
  // explicit Set(const Value ...values) : set_of_values(values...) {}
  std::unordered_set<Value> set_of_values;
};

// A parameter validator, validating a parameter of type value.
template <class Value>
struct Validator {
  /// In ROS, parameters can only be of certain types
  using ROSValue = std::conditional_t<std::is_unsigned_v<Value>, int, Value>;
  /// The type of the predicate
  using Validate = std::function<bool(const ROSValue &)>;

  /// Allow default-constructed validator, by default allowing all values
  /// in the set of values defined by the data type.
  Validator() {
    if constexpr (std::is_unsigned_v<Value>)
      validate = [](const ROSValue &new_value) { return new_value >= 0; };
    else
      validate = [](const ROSValue &) { return true; };
  }

  /// Construct explicitly from a  validation predicate
  explicit Validator(const Validate &validate) : validate(validate) {}

  /// Allow implicit conversion from some easy sets:
  Validator(const Interval<Value> &interval)  /// NOLINT
  {
    validate = [interval](const ROSValue &new_value) {
      return new_value >= interval.minimum && new_value <= interval.maximum;
    };
  }

  /// Implicit conversion from a Set of values
  Validator(const Set<Value> &set)  /// NOLINT
  {
    validate = [set](const ROSValue &new_value) {
      return set.count(new_value) > 0;  /// contains is C++20 :(
    };
  }

  /// A predicate that indicates whether a given value is feasible.
  /// By default, a validator always returns true since the parameter is unconstrained
  Validate validate;
};

/// TODO This is totally unnecessary, use icey::Parameter and only attach it !!
class DynParameterTag {};
template <class Value>
struct DynParameter : public DynParameterTag {
  using type = Value;
  static_assert(is_valid_ros_param_type<Value>::value, "Type is not an allowed ROS parameter type");
  explicit DynParameter(const std::optional<Value> &default_value,
                        const Validator<Value> &validator = Validator<Value>(),
                        std::string description = "", bool read_only = false)
      :  // parameter_name(parameter_name),
        value(default_value),
        default_value(default_value),
        validator(validator),
        description(std::move(description)),
        read_only(read_only) {}

  const Value &get_value() const { return value.value(); }

  /// Allow implicit conversion to the stored value type
  operator Value() const  /// NOLINT
  {
    return get_value();
  }

  std::optional<Value> value;
  std::string parameter_name;
  std::optional<Value> default_value;
  Validator<Value> validator;
  std::string description;
  bool read_only;
};

template <class Derived>
struct ParametersStruct : public Derived {
  friend std::ostream &operator<<(std::ostream &os, ParametersStruct &params) {
    field_reflection::for_each_field(params, [&os](std::string_view field_name, auto &field_value) {
      os << field_name << ": " << field_value << std::endl;
    });
    return os;
  }
};

template <class T>
static void declare_parameter_struct(
    Context &ctx, T &params, const std::function<void(const std::string &)> &notify_callback,
    std::string name_prefix = "") {
  // auto parameters_struct_obs = ctx.create_observable<
  field_reflection::for_each_field(params, [&ctx, notify_callback, name_prefix](
                                               std::string_view field_name, auto &field_value) {
    
    using Field = std::remove_reference_t<decltype(field_value)>;
    std::string field_name_r(field_name);
    field_name_r = name_prefix + field_name_r;
    if constexpr (std::is_base_of_v<DynParameterTag, Field>) {
      std::cout << "In DynParameterTag, registering param " << field_name_r << std::endl;

      using ParamValue = typename Field::type;
      /// TODO register validator
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = field_value.description;
      desc.read_only = field_value.read_only;
      auto param_obs =
          ctx.declare_parameter<ParamValue>(field_name_r, field_value.default_value, desc);
      param_obs.impl()->register_handler(
          [&field_value, param_obs, field_name_r, notify_callback]() {
            field_value.value = param_obs.value();
            notify_callback(field_name_r);
          });
    } else if constexpr (is_valid_ros_param_type<Field>::value) {
      std::cout << "In is_valid_ros_param_type, registering param " << field_name_r << std::endl;

      /// TODO to convert std::vector to std::array, do better
      auto field_value_init = to_ros_param_type(field_value);
      using ParamValue = std::remove_reference_t<decltype(field_value_init)>;
      
      auto param_obs = ctx.declare_parameter<ParamValue>(field_name_r, field_value_init);
      param_obs.impl()->register_handler(
          [&field_value, param_obs, field_name_r, notify_callback]() {
            field_value = param_obs.value();
            notify_callback(field_name_r);
          });
    } else if constexpr (std::is_aggregate_v<Field>) {
      std::cout << "In is_aggregate_v, registering param " << field_name_r << std::endl;
      /// Else recurse for supporting grouped params
      declare_parameter_struct(ctx, field_value, notify_callback, field_name_r + ".");
    } else {
      /// static_assert(false) would always trigger, that is why we use this lambda-workaround, see
      /// https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2022/p2593r0.html
      static_assert(
          []() { return false; },
          "Every field of the parameters struct must be of type T or icey::DynParameter<T> or a "
          "struct of such, where T is a valid ROS param type (see rcl_interfaces/ParameterType)");
    }
  });
}

template <class T>
static void declare_parameter_struct(
    T &params, const std::function<void(const std::string &)> &notify_callback) {
  declare_parameter_struct(get_global_context(), params, notify_callback);
}
}  // namespace icey
