#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavigateToPoseAction : public BT::RosActionNode<NavigateToPose> {
public:
    NavigateToPoseAction(const std::string& name, const BT::NodeConfig& conf,
                         const BT::RosNodeParams& params)
        : RosActionNode<NavigateToPose>(name, conf, params) {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<double>("x", "Target position X"),
            BT::InputPort<double>("y", "Target position Y")
        });
    }

    bool setGoal(Goal& goal) override
    {
        // Načteme souřadnice z Blackboardu
        auto x = getInput<double>("x");
        auto y = getInput<double>("y");
        
        // Pokud některé číslo chybí, vrátíme false (uzel selže ještě před jízdou)
        if (!x || !y) return false;

        // Naplníme cíl zprávy
        goal.pose.header.frame_id = "map";
        goal.pose.pose.position.x = x.value();
        goal.pose.pose.position.y = y.value();
        goal.pose.pose.orientation.w = 1.0;
        
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override
    {
        // Zkontrolujeme, jestli navigace uspěla (robot dojel)
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
    {
        // Zalogujeme chybu a vrátíme FAILURE
        std::cout << "[NavigateToPoseAction] Jizda selhala. Kod chyby: " << static_cast<int>(error) << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/) override
    {
        // Udržujeme uzel ve stavu RUNNING po celou dobu jízdy robota
        return BT::NodeStatus::RUNNING;
    }
};

CreateRosNodePlugin(NavigateToPoseAction, "NavigateToPoseAction");
