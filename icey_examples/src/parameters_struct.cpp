#include <icey/icey.hpp>

/// Here you declare in a single struct all parameters of the node:
struct NodeParameters {
  /// We can have regular fields :
  double amplitude{3};

  /// And as well parameters with constraints and a description:
  icey::ParameterStream<double> frequency{10., icey::Interval(0., 25.),
                                       std::string("The frequency of the sine")};
  
  icey::ParameterStream<std::string> mode{"single", icey::Set<std::string>({"single", "double", "pulse"})};
  /// We can also have nested structs with more parameters, they will be named others.max_amp, others.cov:
  struct OtherParams {
    double max_amp = 6.;
    std::vector<double> cov;
  } others;
};

int main(int argc, char **argv) {  
  auto node = icey::create_node<icey::Node>(argc, argv, "parameters_struct_example");
  
  /// Now create an object of the node-parameters that will be updated:
  NodeParameters params;

  /// Now simply declare the parameter struct and a callback that is called when any field updates: 
  node->icey().declare_parameter_struct(params, [&](const std::string &changed_parameter) {
        RCLCPP_INFO_STREAM(node->get_logger(),
                           "Parameter " << changed_parameter << " changed");
      });

  icey::spin(node);
}