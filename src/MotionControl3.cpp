#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <thread>

MotionControlNode::MotionControlNode() : rclcpp::Node("motion_control_node") {
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");

    nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
        this, "/go_to_goal",
        std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
        std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));

    while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(get_logger(), "Čekám na plánovač...");
    }
    RCLCPP_INFO(get_logger(), "Motion control připraven k jízdě.");
}

void MotionControlNode::checkCollision() {
    // Přidána pojistka (|| collision_stop_): Pokud už robot stojí kvůli zdi, 
    // nepokračuj, aby terminál nebyl zaspamovaný tisíci hláškami.
    if (laser_scan_.ranges.empty() || collision_stop_) return;

    double thresh = 0.8;
    int total = laser_scan_.ranges.size();
    int center = total / 2;
    int search_range = total / 3;

    for (int i = center - search_range; i <= center + search_range; ++i) {
        if (i < 0 || i >= total) continue;
        if (std::isfinite(laser_scan_.ranges[i]) && laser_scan_.ranges[i] < thresh) {
            
            // JEDNORÁZOVÝ VÝPIS O NÁRAZU:
            RCLCPP_ERROR(get_logger(), "!!! NOUZOVÉ ZASTAVENÍ !!! Robot narazil na překážku (vzdálenost: %.2f m)", laser_scan_.ranges[i]);

            collision_stop_ = true;

            // Zastav vždy bez ohledu na stav goal_handle
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            twist_publisher_->publish(stop);

            // Abortuj pokud existuje aktivní cíl
            if (goal_handle_ && goal_handle_->is_executing()) {
                auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
                goal_handle_->abort(result);
            }
            return;
        }
    }
}

void MotionControlNode::updateTwist() {
    if (collision_stop_) {
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        return;
    }

    if (!goal_handle_ || !goal_handle_->is_executing() || path_.poses.empty()) return;

    double rx = current_pose_.pose.position.x;
    double ry = current_pose_.pose.position.y;
    double yaw = tf2::getYaw(current_pose_.pose.orientation);

    double lookahead = 0.4;
    double v_max = 0.15;
    double w_max = 0.8;

    // Dynamické zpomalení podle nejbližší překážky vpředu
    if (!laser_scan_.ranges.empty()) {
        int total = laser_scan_.ranges.size();
        int center = total / 2;
        int range = total / 6;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = center - range; i <= center + range; ++i) {
            if (i >= 0 && i < total && std::isfinite(laser_scan_.ranges[i]))
                min_dist = std::min(min_dist, (double)laser_scan_.ranges[i]);
        }
        // Plná rychlost > 1.0m, nulová < 0.55m
        if (min_dist < 1.0) {
            double factor = std::clamp((min_dist - 0.55) / (1.0 - 0.55), 0.0, 1.0);
            v_max *= factor;
        }
    }

    // Odstraň waypointy, které už robot minul
    while (path_.poses.size() > 1) {
        double d = std::hypot(path_.poses.front().pose.position.x - rx,
                              path_.poses.front().pose.position.y - ry);
        if (d < lookahead) path_.poses.erase(path_.poses.begin());
        else break;
    }

    // Cílový bod = první bod dál než lookahead, jinak poslední waypoint
    geometry_msgs::msg::Pose target = path_.poses.back().pose;
    for (const auto& wp : path_.poses) {
        double d = std::hypot(wp.pose.position.x - rx, wp.pose.position.y - ry);
        if (d >= lookahead) { target = wp.pose; break; }
    }

    double dx = target.position.x - rx;
    double dy = target.position.y - ry;
    double alpha = std::atan2(dy, dx) - yaw;
    while (alpha >  M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    geometry_msgs::msg::Twist twist;

    double alpha_thresh = M_PI / 4.0;
    if (std::abs(alpha) > alpha_thresh) {
        twist.linear.x = 0.0;
    } else {
        twist.linear.x = v_max * (1.0 - std::abs(alpha) / alpha_thresh);
    }
    twist.angular.z = std::clamp((2.0 * v_max * std::sin(alpha)) / lookahead, -w_max, w_max);

    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    collision_stop_ = false; // reset při novém úkolu
    goal_handle_ = goal_handle;
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal = goal_handle->get_goal()->pose;
    plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    rclcpp::Rate loop_rate(10.0);
    while (rclcpp::ok() && goal_handle_ && goal_handle_->is_active()) {
        if (goal_handle_->is_canceling()) return;

        double dist = std::hypot(
            goal_handle_->get_goal()->pose.pose.position.x - current_pose_.pose.position.x,
            goal_handle_->get_goal()->pose.pose.position.y - current_pose_.pose.position.y);

        if (dist < 0.2) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            goal_handle_->succeed(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
            RCLCPP_INFO(get_logger(), "Cíle úspěšně dosaženo!");
            return;
        }
        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    auto res = future.get();
    if (res && !res->plan.poses.empty()) {
        path_ = res->plan;
        RCLCPP_INFO(get_logger(), "Trasa přijata, vyrážím.");
        goal_handle_->execute();
        std::thread(&MotionControlNode::execute, this).detach();
    } else {
        RCLCPP_ERROR(get_logger(), "Chyba: Plánovač nenašel cestu!");
        goal_handle_->execute();
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
    }
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    current_pose_.header = msg.header;
    current_pose_.pose = msg.pose.pose;

    checkCollision(); // volá se vždy

    if (collision_stop_) {
        // Aktivně drž nulovou rychlost
        geometry_msgs::msg::Twist stop;
        twist_publisher_->publish(stop);
        return;
    }

    if (goal_handle_ && goal_handle_->is_executing()) updateTwist();
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
}
