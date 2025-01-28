/// This example shows how to initialize the data flow graph based on
/// parameters, and managing algorithm state, for example loading a neural network model from file
#include <icey/icey.hpp>

#include "std_msgs/msg/float32.hpp"

/// Here, you could load your neural network model from file, from ONNX, TFLite etc.
void load_neural_network_model(const std::string& filename) { 
  (void)filename; 
  std::cout << "Loeaded NN." << std::endl; 
}

int main(int argc, char **argv) {
  icey::icey_debug_print = true;

  auto nn_model_filename_param = icey::declare_parameter<std::string>("nn_model_filename", "");
  auto publish_debug_markers_param = icey::declare_parameter<bool>("publish_debug_markers", false);

  /// There are pre-defined events on which we can register, this callback is called immediatelly
  /// after all parameters are available, but before everything else (timers, subscribers,
  /// publishers etc.) is initialized. Note that it does not matter where we call this, since here
  /// we are still in the declaration phase.
  icey::after_parameter_initialization([&]() {
    load_neural_network_model(nn_model_filename_param.value());

    /// Add additional subscribers
    auto map_base_link_tf = icey::create_transform_subscription("map", "base_link");

    /// TODO
    auto rectangle_sig = map_base_link_tf.then([](auto) {
      std::optional<std_msgs::msg::Float32> result;
      return result;
    });

    /// Now conditionally create a publisher based on the parameters:
    if (publish_debug_markers_param.value()) {
      rectangle_sig.publish("debug_marker");
    }
  });

  /// Register a callback that gets called in the Node's destructor
  icey::on_node_destruction([] {
    std::cout << "Cleaned  up NN. " << std::endl;
    // clean_up_c_library()
  });
  icey::spawn(argc, argv, "signal_generator_example");
}