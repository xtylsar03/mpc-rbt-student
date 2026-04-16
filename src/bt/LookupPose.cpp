#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include <map>
#include <string>

struct Pose2D {
    double x;
    double y;
};

class LookupPose : public BT::SyncActionNode {
public:
    LookupPose(const std::string& name, const BT::NodeConfig& config)
        : SyncActionNode(name, config)
    {
        // DOPLNĚNO: Naplnění tabulky souřadnic ze zadání
        pose_table_["1"] = {4.5, 1.5};
        pose_table_["2"] = {4.5, -0.5};
        pose_table_["3"] = {4.5, -2.5};

        pose_table_["A1"] = {1.5, 0.5};   pose_table_["A2"] = {1.5, -1.5};
        pose_table_["B1"] = {-0.5, 0.5};  pose_table_["B2"] = {-0.5, -1.5};
        pose_table_["C1"] = {-2.5, 0.5};  pose_table_["C2"] = {-2.5, -1.5};
        pose_table_["D1"] = {-4.5, 0.5};  pose_table_["D2"] = {-4.5, -1.5};
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("location_id", "Location ID to look up"),
            BT::OutputPort<double>("x", "Looked up X coordinate"),
            BT::OutputPort<double>("y", "Looked up Y coordinate")
        };
    }

    BT::NodeStatus tick() override
    {
        // DOPLNĚNO: Načtení ID a vyhledání souřadnic
        auto id = getInput<std::string>("location_id");
        if (!id) return BT::NodeStatus::FAILURE;

        auto it = pose_table_.find(id.value());
        if (it == pose_table_.end()) {
            return BT::NodeStatus::FAILURE;
        }

        setOutput("x", it->second.x);
        setOutput("y", it->second.y);
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::map<std::string, Pose2D> pose_table_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<LookupPose>("LookupPose");
}
