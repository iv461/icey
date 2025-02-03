/// A bag of template metaprogramming tricks for C++17, detecting types, operating on lists of types
/// etc. For some parts of ICEY such as the synchronizer, we have to operate on type lists, but
/// luckily not too much.
#pragma once

#include <functional>
#include <memory>
#include <array>
#include <optional>
#include <tuple>
#include <type_traits>
#include <variant>
namespace icey {

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
struct is_optional : std::false_type {};

template <class T>
struct is_optional<std::optional<T>> : std::true_type {};

template <class T>
constexpr bool is_optional_v = is_optional<T>::value;

template <typename... Args>
struct is_variant : std::false_type {};

template <typename... Args>
struct is_variant<std::variant<Args...>> : std::true_type {};

template <typename... Args>
constexpr bool is_variant_v = is_variant<Args...>::value;

template <class T>
struct t_is_std_array : std::false_type{};

template <class T, std::size_t N>
struct t_is_std_array< std::array<T, N> > : std::true_type{};

template <class T>
constexpr bool is_std_array = t_is_std_array<T>::value;

template <class T>
struct t_is_shared_ptr : std::false_type {};

template <class T>
struct t_is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <class T>
constexpr bool is_shared_ptr = t_is_shared_ptr<T>::value;

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

/// Are all the types of the tuple the same ?
template <class Head, typename... Tail>
constexpr bool all_same(const std::tuple<Head, Tail...> &) {
  return (std::is_same_v<Head, Tail> && ...);
}

/// TODO consider using hana::unpack and hana::fuse
template <class Func, class Tuple>
auto apply_if_tuple(Func &&func, Tuple &&tuple) {
  if constexpr (is_tuple_v<std::decay_t<Tuple>> || is_pair_v<std::decay_t<Tuple>>) {
    // Tuple detected, unpack and call the function
    return std::apply(std::forward<Func>(func), std::forward<Tuple>(tuple));
  } else {
    // Not a tuple, just call the function directly
    return func(std::forward<Tuple>(tuple));
  }
}

}  // namespace icey