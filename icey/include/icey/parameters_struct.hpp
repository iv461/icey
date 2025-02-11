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

/// @brief Declare all fields of a given parameter struct as ROS parameters.
/// @tparam T the type of the Parameter struct. It is a struct with fields of either a primitive type supported by ROS (e.g. `double`) or a `icey::ParameterStream`, or another (nested) struct with more such fields.
/// @param ctx the icey-Context, required for registering the parameters.
/// @param params The instance of the parameter struct where the values will be written to.
/// @param notify_callback The callback that gets called when any field changes
/// @param name_prefix Prefix for each parameter. Used by the recursive call to support nested structs.
template <class T>
static void declare_parameter_struct(
    Context &ctx, T &params, const std::function<void(const std::string &)> &notify_callback,
    std::string name_prefix = "") {
  
  field_reflection::for_each_field(params, [&ctx, notify_callback, name_prefix](
                                               std::string_view field_name, auto &field_value) {
    using Field = std::remove_reference_t<decltype(field_value)>;
    std::string field_name_r(field_name);
    if constexpr (std::is_base_of_v<StreamTag, Field>) {
      field_value.impl()->parameter_name = field_name_r;
      field_value.impl()->register_handler([field_name_r, notify_callback](const auto &new_state) {
            notify_callback(field_name_r);
          });
      field_value.register_with_ros(*ctx.node);
    } else if constexpr (is_valid_ros_param_type<Field>::value) {
      using ParamValue = std::remove_reference_t<decltype(field_value)>;
      ctx.declare_parameter<ParamValue>(field_name_r, field_value)
        .impl()->register_handler(
            [&field_value, field_name_r, notify_callback](const auto &new_state) {
              field_value = new_state.value();
              notify_callback(field_name_r);
            });
    } else if constexpr (std::is_aggregate_v<Field>) {
      /// Else recurse for supporting grouped params
      declare_parameter_struct(ctx, field_value, notify_callback, name_prefix + std::string(field_name) + ".");
    } else {
      /// static_assert(false) would always trigger, that is why we use this workaround, see
      /// https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2022/p2593r0.html
      static_assert(
          std::is_array_v<int>, 
          "Every field of the parameters struct must be of type T or icey::Parameter<T> or a "
          "struct of such, where T is a valid ROS param type (see rcl_interfaces/ParameterType)");
    }
  });
}

}  // namespace icey
