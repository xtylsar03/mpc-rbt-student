#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

// DOPLNĚNO: Třída BTServer
class BTServer : public BT::TreeExecutionServer {
public:
    BTServer(const rclcpp::NodeOptions& options)
        : TreeExecutionServer(options) {}

    void onTreeCreated(BT::Tree& tree) override {
        // Volitelně logger pro ladění v terminálu
        // tree.addLogger(std::make_unique<BT::StdCoutLogger>(tree));
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // DOPLNĚNO: Instance serveru a spuštění executoru
    rclcpp::NodeOptions options;
    auto server = std::make_shared<BTServer>(options);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(server->node());
    
    RCLCPP_INFO(server->node()->get_logger(), "BT Server is running...");
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
