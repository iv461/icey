#include <boost/mp11.hpp>

#include <boost/hana.hpp>

#include <type_traits>
#include <boost/type_index.hpp>

#include <tuple>
#include <iostream>

namespace mp = boost::mp11;
namespace hana = boost::hana;

struct Base {
    virtual ~Base(){

    }
};
struct Derived1 : Base {};
struct Derived2 : Base {};
struct NotDerived {
    virtual ~NotDerived(){}
};
struct NotDerived2 {
    virtual ~NotDerived2(){}
};
struct NotDerived3 {
    virtual ~NotDerived3(){}
};

//template<class T>
//using is_based = std::is_base_of<Base, T>;

/*
template<class T>
auto filter(T tup) {
    return std::apply([&](auto first, auto... rest) {
        auto filtered_rest = [&]{
            if constexpr (sizeof...(rest)) {
                return filter(std::tuple{rest...});
            } else {
                return std::tuple{};
            }
        }();

        if constexpr ( is_based < decltype(first)  > :: value ) {
            return std::tuple_cat(std::tuple{first}, filtered_rest);
        } else {
            return filtered_rest;
        }
    }, tup);
}

template<typename... Args> 
void partition_args(){
     // Define a tuple of types
    //using types = std::tuple<Base, Derived1, NotDerived2, Derived2, NotDerived>;
    //using types = mp::mp_list<Base, Derived1, NotDerived2, Derived2, NotDerived>;
    using types = mp::mp_list<Args ...>;

    auto t = std::tuple<Args...>(Args{}...);
    auto satisfying_values = filter(t);
    
    using TupleTypeList = mp::mp_transform<std::decay_t, types>;
    
    using Partitioned = mp::mp_partition<types, is_based>;


    using SatisfyingTuple =  mp::mp_first<Partitioned>;
    using NonSatisfyingTuple = class mp::mp_second<Partitioned>;


    // Convert filtered types to a std::tuple
    using SatisfyingSTDTuple = mp::mp_apply<std::tuple, SatisfyingTuple>;
    using NonSatisfyingSTDTuple = mp::mp_apply<std::tuple, NonSatisfyingTuple>;

    static_assert ( std::is_same_v <  std::tuple<Base, Derived1, Derived2>, decltype(satisfying_values) > );
    //static_assert( std::is_same_v < SatisfyingTuple, std::tuple<Base, Derived1, Derived2> >);
    //static_assert( std::is_same_v < NonSatisfyingSTDTuple, std::tuple<NotDerived2, NotDerived> >);

    std::cout << "Result: "<< std::endl;

    std::cout << "Types derived from base: "<< std::endl;
    // Print type names (optional)
    mp::mp_for_each<SatisfyingSTDTuple>([](auto t) {
        std::cout << typeid(t).name() << std::endl;
    });
    
    std::cout << "Types NOT derived from base: "<< std::endl;
    // Print type names (optional)
    mp::mp_for_each<NonSatisfyingSTDTuple>([](auto t) {
        std::cout << typeid(t).name() << std::endl;
    });
}
*/

/*template <typename T>
struct basic_type {};

template <typename T>
constexpr auto is_based(basic_type<T> const&)
{ return hana::bool_c<false>; }
 
template <typename T>
constexpr auto is_based(basic_type<Base> const&)
{ return hana::bool_c<true>; }
*/

template <typename T>
constexpr auto is_based(hana::basic_type<T>) {
    if constexpr(std::is_base_of_v<Base, T >) 
        return hana::bool_c<true>;
    else 
        return hana::bool_c<false>;    
}
 

template<typename... Args> 
void partition_tuple_hana(std::tuple<Args...> tuple){

    auto Types = hana::tuple_t<Args...>;

    auto DerivedOnes = hana::remove_if(Types, [](auto t) { return not is_based(t); });

    //hana::make_tuple(
    auto NotDerivedOnes = hana::remove_if(Types, [](auto t) { return is_based(t); });


    std::cout << "Result: "<< std::endl;

    std::cout << "Types derived from base: "<< std::endl;
    
    hana::for_each(DerivedOnes, [](auto const& element) {
        std::cout << boost::typeindex::type_id_runtime(element).pretty_name() << std::endl;
    });
    
    std::cout << "Types NOT derived from base: "<< std::endl;

    hana::for_each(NotDerivedOnes, [](auto const& element) {
        std::cout << boost::typeindex::type_id_runtime(element).pretty_name() << std::endl;
    });

    std::cout << "Done. "<< std::endl;
    
}

int main() {
    std::cout << "MP " << std::endl;
    std::tuple<Base, Derived1, NotDerived3, Derived2, NotDerived, NotDerived2> tup{};

    partition_tuple_hana(tup);

    //partition_args<Base, Derived1, Derived2, NotDerived>();
    return 0;
}