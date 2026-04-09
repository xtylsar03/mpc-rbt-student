#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"
#include <tf2/utils.h> 
#include <cmath>

MotionControlNode::MotionControlNode() : rclcpp::Node("motion_control_node") {

    // Subscribers for odometry and laser scans
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
    
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

    // Publisher for robot control
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Client for path planning
    plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

    // Action server
    nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
        this,
        "/go_to_goal",
        std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
        std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Motion control node started.");

    // Connect to path planning service server
    while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(get_logger(), "Waiting for plan_path service...");
    }
}

void MotionControlNode::checkCollision() {
    if (laser_scan_.ranges.empty() || !goal_handle_ || !goal_handle_->is_active()) return;

    double thresh = 0.3; // Bezpečnostní vzdálenost
    int center = laser_scan_.ranges.size() / 2;
    int search_range = laser_scan_.ranges.size() / 6;

    for (int i = center - search_range; i <= center + search_range; ++i) {
        if (std::isfinite(laser_scan_.ranges[i]) && laser_scan_.ranges[i] < thresh) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            
            RCLCPP_ERROR(get_logger(), "KOLIZE HROZÍ! Nouzové zastavení.");
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
            break;
        }
    }
}

void MotionControlNode::updateTwist() {
    // Počítáme pouze pokud akce běží a máme trasu (tu zelenou čáru)
    if (!goal_handle_ || !goal_handle_->is_executing() || path_.poses.empty()) return;

    double robot_x = current_pose_.pose.position.x;
    double robot_y = current_pose_.pose.position.y;
    double robot_yaw = tf2::getYaw(current_pose_.pose.orientation);

    // PARAMETRY ŘÍZENÍ
    double lookahead_distance = 0.6; // Zvětšeno pro plynulejší jízdu a potlačení oscilací
    double max_v = 0.3; 
    double max_w = 0.8; // Mírně sníženo, aby se robot netočil tak agresivně

    // Hledání cílového bodu na trase (Pure Pursuit)
    geometry_msgs::msg::Pose target_point = path_.poses.back().pose;
    bool point_found = false;

    for (const auto& waypoint : path_.poses) {
        double dist = std::hypot(waypoint.pose.position.x - robot_x, waypoint.pose.position.y - robot_y);
        if (dist >= lookahead_distance) {
            target_point = waypoint.pose;
            point_found = true;
            break;
        }
    }

    // Výpočet úhlu k cíli
    double dx = target_point.position.x - robot_x;
    double dy = target_point.position.y - robot_y;
    double alpha = std::atan2(dy, dx) - robot_yaw;
    
    // Normalizace úhlu do rozsahu <-PI, PI>
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    geometry_msgs::msg::Twist twist;

    // Dopředná rychlost (zpomaluje v ostrých zatáčkách)
    twist.linear.x = max_v * std::cos(alpha);
    if (twist.linear.x < 0.0) twist.linear.x = 0.0; 
    
    // Úhlová rychlost (Pure Pursuit vzorec)
    twist.angular.z = (2.0 * max_v * std::sin(alpha)) / lookahead_distance;

    // Saturace úhlové rychlosti
    if (twist.angular.z > max_w) twist.angular.z = max_w;
    if (twist.angular.z < -max_w) twist.angular.z = -max_w;

    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) 
{
    (void)uuid; (void)goal;
    RCLCPP_INFO(get_logger(), "Přijat nový požadavek na navigaci.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) 
{
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Akce zrušena uživatelem.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) 
{
    goal_handle_ = goal_handle;

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start.pose = current_pose_.pose; 
    request->start.header.frame_id = "map";
    request->goal = goal_handle->get_goal()->pose; 

    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    rclcpp::Rate loop_rate(10.0); 
    auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

    while (rclcpp::ok() && goal_handle_ && goal_handle_->is_active()) {

        if (goal_handle_->is_canceling()) {
            goal_handle_->canceled(result);
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            return;
        }

        if (!path_.poses.empty()) {
            auto final_pose = path_.poses.back().pose.position;
            double dist_to_goal = std::hypot(final_pose.x - current_pose_.pose.position.x, 
                                             final_pose.y - current_pose_.pose.position.y);
            
            if (dist_to_goal < 0.2) { 
                geometry_msgs::msg::Twist stop;
                twist_publisher_->publish(stop);
                goal_handle_->succeed(result);
                RCLCPP_INFO(get_logger(), "Úspěšně dosaženo cíle!");
                return;
            }
        }

        feedback->current_pose = current_pose_;
        goal_handle_->publish_feedback(feedback);

        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto response = future.get();
    
    if (response && response->plan.poses.size() > 0) {
        path_ = response->plan;
        RCLCPP_INFO(get_logger(), "Trasa přijata (%ld bodů). Spouštím sledování...", path_.poses.size());
        goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        RCLCPP_ERROR(get_logger(), "Nepodařilo se naplánovat trasu k cíli!");
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    current_pose_.pose = msg.pose.pose;
    current_pose_.header = msg.header;

    checkCollision();
    
    // Publikujeme rychlosti pouze pokud akce skutečně běží
    if (goal_handle_ && goal_handle_->is_executing()) {
        updateTwist();
    }
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
}
