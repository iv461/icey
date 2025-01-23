/// This header provides support to define a set of ROS parameters
/// including constraints and get updates if of them changes. 
/// This greatly simplifies the declaration of many parameters 
/// and comes close to the dynamic_reconfigure package in ROS 1.
/// The struct is declared at compile time and uses the Hana library to do static reflection in C++17.
/// We need the names of the fields and be able to access them by name (i.e. param.my_param) while knowing the type, and also 
/// have the field names as runtime-strings.
/// Note that in C++17 this is impossible to do without macros, only in C++20 it became possible.
#pragma once 

#include <type_traits>
#include <functional>
#include <unordered_set>
#include <optional>
#include <string>
namespace icey {
    /// First, some constraints we can impose on parameters:
    /// A closed interval, meaning a value must be greater or equal to a minimum value 
    /// and less or equal to a maximal value.
    template<class Value>
    struct Interval {
        static_assert(std::is_arithmetic_v<Value>, "The value type must be a number");
        Interval(Value minimum, Value maximum) : minimum(minimum), maximum(maximum) {}
        Value minimum;
        Value maximum;
    };  

    /// A set specified by a given list of values that are in the set.
    template<class Value>
    struct Set { 
        
        //explicit Set(const Value ...values) : set_of_values(values...) {}
        std::unordered_set<Value> set_of_values;
    };

    // A parameter validator, validating a parameter of type value.
    template<class Value>
    struct Validator {
        /// In ROS, parameters can only be of certain types
        using ROSValue = std::conditional_t<std::is_unsigned_v<Value>, int, Value>;
        /// The type of the predicate 
        using Validate = std::function<bool(const ROSValue&)>;

        /// Allow default-constructed validator, by default allowing all values 
        /// in the set of values defined by the data type.
        Validator() {
            if constexpr(std::is_unsigned_v<Value>)
                validate = [](const ROSValue &new_value) {return new_value >= 0;};
            else
                validate = [](const ROSValue &new_value) {return true;};
        }

        /// Construct from an explicit validation predicate
        Validator(const Validate &validate) : validate(validate) {}

        /// Allow implicit conversion from some easy sets: 
        Validator(const Interval<Value> &interval) {
            validate = [interval](const ROSValue &new_value) {
                    return new_value >= interval.minimum && new_value <= interval.maximum;
            };
        }
        
        /// Implicit conversion from 
        Validator(const Set<Value> &set) {
            validate = [set](const ROSValue &new_value) {
                    return set.count(new_value) > 0; /// contains is C++20 :(
            };
        }

        /// A predicate that indicates whether a given value is feasible.
        /// By default, a validator always returns true since the parameter is unconstrained
        Validate validate;
    };

    template<class Value>
    struct DynParameter {

        DynParameter(const std::optional<Value> &default_value,
                    const Validator<Value> &validator = Validator<Value>(), 
                    const std::string &description = "", bool is_dynamic=true):
                    //parameter_name(parameter_name),
                    value(default_value),
                    default_value(default_value),
                    validator(validator),
                    description(description),
                    is_dynamic(is_dynamic) {
            
        }   

        /// Allow implicit conversion to the stored value type
        operator Value() const { return value.value(); }
        const Value &get_value() const  {return value.value();}

        std::optional<Value> value;
        std::string parameter_name; 
        std::optional<Value> default_value;
        Validator<Value> validator;
        std::string description;
        bool is_dynamic;
    };

    struct DynParameters {

    };
}
