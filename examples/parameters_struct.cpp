//#include <icey/icey.hpp>

#include <icey/parameters_struct.hpp>

#include <icey/impl/reflect.hpp>
#include <iostream>
struct MyParameters {
    //cey::DynParameter<float> max_velocity{2.f, icey::Interval(0.f, 25.f)};
    int a; 
    float b;
};

int main(int argc, char **argv) {
    
    MyParameters f;
    
    // visit each field
field_reflection::for_each_field(f, [](std::string_view field, auto& value) {
    // i: 287
    // d: 3.14
    // hello: Hello World
    // arr: [1, 2, 3]
    // map: {"one": 1, "two": 2}
    std::cout << field << ", " << value << std::endl;
});
    //icey::spawn(argc, argv, "tf_listener_example"); /// Create and start node
}