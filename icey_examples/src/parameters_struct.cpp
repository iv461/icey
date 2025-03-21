
#include <icey/icey.hpp>

/// Here you declare in a single struct all parameters of the node:
struct NodeParameters {
  /// We can have regular fields :
  double amplitude{3};

  /// And as well parameters with constraints and a description:
  icey::Parameter<double> frequency{10., icey::Interval(0., 25.),
                                    std::string("The frequency of the sine")};

  icey::Parameter<std::string> mode{"single",
                                    icey::Set<std::string>({"single", "double", "pulse"})};
  /// We can also have nested structs with more parameters, they will be named others.max_amp,
  /// others.cov:
  struct OtherParams {
    double max_amp = 6.;
    std::vector<double> cov;
  } others;
};

class MyNode : public icey::Node {
public: 
  MyNode(std::string name) : icey::Node(name) {
    /// Now simply declare the parameter struct and a callback that is called when any field updates:
    icey().declare_parameter_struct(params_, [this](const std::string &changed_parameter) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Parameter " << changed_parameter << " changed");
    });
  }

  /// Store the parameters as a class member: 
  NodeParameters params_;
};

int main(int argc, char **argv) {
  auto node = icey::create_node<MyNode>(argc, argv, "icey_parameters_struct_example");
  icey::spin(node);
}