/// A bag of template metaprogramming tricks for C++17, detecting types, operating on lists of types etc. 
/// For some parts of ICEY such as the synchronizer, we have to operate on type lists, but luckily not too much.
#pragma once 

#include <functional>
#include <tuple>
#include <optional>
#include <type_traits>

namespace icey {

template<typename T>
struct is_tuple : std::false_type {};

template<typename... Args>
struct is_tuple<std::tuple<Args...>> : std::true_type {};

template<typename T>
struct is_pair : std::false_type {};

template<typename... Args>
struct is_pair<std::pair<Args...>> : std::true_type {};

template<typename T>
constexpr bool is_tuple_v = is_tuple<T>::value;

template<typename T>
constexpr bool is_pair_v = is_pair<T>::value;

template<class T>
struct is_optional : std::false_type {};

template<class T>
struct is_optional<std::optional<T>> : std::true_type {};

template<class T>
constexpr bool is_optional_v = is_optional<T>::value;

template<class T>
struct t_is_shared_ptr : std::false_type {};

template<class T>
struct t_is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template<class T>
constexpr bool is_shared_ptr = t_is_shared_ptr<T>::value;

template<class T>
struct remove_optional { using type = T;};

template<class T>
struct remove_optional<std::optional<T>> { using type = T; };

template<class T>
using remove_optional_t = typename remove_optional<T>::type;

template<class T>
struct remove_shared_ptr { using type = T;};

template<class T>
struct remove_shared_ptr<std::shared_ptr<T>> { using type = T; };

template<class T>
using remove_shared_ptr_t = typename remove_shared_ptr<T>::type;

/// Are all the types of the tuple the same ?
template<typename Head, typename...Tail>
constexpr bool all_same(const std::tuple<Head, Tail...>&){
    return (std::is_same_v<Head,Tail> && ...);
}

template<typename Func, typename Tuple>
auto apply_if_tuple(Func&& func, Tuple&& tuple) {
    if constexpr (is_tuple_v<std::decay_t<Tuple>> || is_pair_v<std::decay_t<Tuple>>) {
        // Tuple detected, unpack and call the function
        return std::apply(std::forward<Func>(func), std::forward<Tuple>(tuple));
    } else {
        // Not a tuple, just call the function directly
        return func(std::forward<Tuple>(tuple));
    }
}

}