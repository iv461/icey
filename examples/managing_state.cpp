/// This example shows how to store additional ROS entities in the node 
/// using the functional API. It also shows how every member you would put in your node class
/// you can simply have as a local variable and capture it. 

#include <icey/icey_ros2.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "std_msgs/msg/float32.hpp"

/// Here, you could load your neural network model from file, from ONNX, TFLite etc.
void load_neural_network_model(std::string filename) {
    
}

int main(int argc, char **argv) {

    /// As an additional ROS entity, we use a diagnostcs updater as an example.
    std::shared_ptr<diagnostic_updater::Updater> diagnostics_updater;

    auto nn_model_filename_param = icey::declare_parameter<std::string>("nn_model_filename", "");
    auto publish_debug_markers_param = icey::declare_parameter<bool>("publish_debug_markers", false);

    /// After parameter initialization, we can in
    icey::after_parameter_initialization([&] () {

        diagnostics_updater = std::make_shared<diagnostic_updater::Updater>(icey::node);

        
    });

    /// Register a callback that gets called in the Node's destructor
    icey::on_node_destruction([] {
        // clean_up_c_library() 
    });
    icey::spawn(argc, argv, "managing_state_example"); 
}