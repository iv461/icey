
#include <boost/hana.hpp>
#include <boost/hana/ext/std/tuple.hpp> 

#include <type_traits>
#include <boost/type_index.hpp>
#include <tuple>
#include <iostream>
#include <variant>

namespace hana = boost::hana;

struct Base {
    int a{1};
};
struct Derived1 : Base {};
struct Derived2 : Base {};
struct NotDerived {
    int a{6};
};
struct NotDerived2 {
    int a{7};
};

struct NotDerived3 {
    int a{3};
};

template <typename T>
//constexpr auto is_based(hana::basic_type<T>) { type version
constexpr auto is_based(T) { /// value version
    if constexpr(std::is_base_of_v<Base, T >)
        return hana::bool_c<true>;
    else 
        return hana::bool_c<false>;    
}
 
void f(int i) {
    std::cout << "f(i): " << i << std::endl;
}


template<typename... Args> 
void partition_tuple_hana(Args... args){

    // hana::unpack(3, f); 

    //auto Types = hana::tuple_t<Args...>;
    auto Types = std::make_tuple(args...);

    auto DerivedOnes = hana::remove_if(Types, [](auto t) { return not is_based(t); });
    auto NotDerivedOnes = hana::remove_if(Types, [](auto t) { return is_based(t); });

    const bool all_are_derived = hana::all_of(DerivedOnes,  [](auto t) { return is_based(t); }); 
    std::cout << "All are derived "<< all_are_derived << std::endl;

    std::cout << "Result: "<< std::endl;
    std::cout << "Types derived from base: "<< std::endl;
    hana::for_each(DerivedOnes, [](auto const& element) {
        std::cout << boost::typeindex::type_id_runtime(element).pretty_name() << std::endl;
    });
    
    std::cout << "Types NOT derived from base: "<< std::endl;
    hana::for_each(NotDerivedOnes, [](auto const& element) {
        std::cout << boost::typeindex::type_id_runtime(element).pretty_name() 
            << ", has value: " << element.a << std::endl;
    });

    std::cout << "Done. "<< std::endl;
}

void range_test() {
    constexpr auto last_r = hana::int_c<5>;
    
    constexpr auto indices = hana::to<hana::tuple_tag>(hana::range_c<std::size_t, 0, 5>);
    auto ret = hana::transform(indices, [&](auto I) {
        std::cout << "index : " << I << std::endl;
        return I;
    });

    hana::for_each(ret, [&](auto I) {
        std::cout << "The Rettich is " << I << std::endl;
    });
    
    
}

int main() {
    std::cout << "MP " << std::endl;
    std::tuple<Base, Derived1, NotDerived3, Derived2, NotDerived, NotDerived2> tup{};

    partition_tuple_hana(Base{}, Derived1{}, NotDerived3{}, Derived2{}, NotDerived{}, NotDerived2{});

    range_test();
    //partition_args<Base, Derived1, Derived2, NotDerived>();
    return 0;
}