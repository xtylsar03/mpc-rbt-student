#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <cmath>

LocalizationNode::LocalizationNode() :
    rclcpp::Node("localization_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

    last_time_ = this->get_clock()->now();

    // Start na x = -0.5, pohled vpřed
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    odometry_.pose.pose.position.x = -0.5;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.position.z = 0.0;
    odometry_.pose.pose.orientation.w = 1.0;

    // Subscriber
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );

    // Publishers
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Statická transformace pro Lidar
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped static_t;
    static_t.header.stamp = this->get_clock()->now();
    static_t.header.frame_id = "base_link";
    static_t.child_frame_id = "Hokuyo_URG_04LX_UG01";
    static_t.transform.translation.x = 0.2;
    static_t.transform.translation.y = 0.0;
    static_t.transform.translation.z = 0.1;
    static_t.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(static_t);
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    
    if (dt <= 0.0 || dt > 1.0) {
        last_time_ = current_time;
        return;
    }
    last_time_ = current_time;

    
    updateOdometry(msg.velocity[1], msg.velocity[0], dt);
    publishOdometry();
    publishTransform();
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    double left_linear  = left_wheel_vel  * robot_config::WHEEL_RADIUS;
    double right_linear = right_wheel_vel * robot_config::WHEEL_RADIUS;

    double linear  = (right_linear + left_linear) / 2.0;
    double angular = (right_linear - left_linear) / (2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS);

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    odometry_.pose.pose.position.x += linear * std::cos(theta) * dt;
    odometry_.pose.pose.position.y += linear * std::sin(theta) * dt;
    theta += angular * dt;

    theta = std::atan2(std::sin(theta), std::cos(theta));

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    odometry_.twist.twist.linear.x  = linear;
    odometry_.twist.twist.angular.z = angular;
}

// TYTO FUNKCE TI V KÓDU CHYBĚLY:
void LocalizationNode::publishOdometry() {
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id  = "base_link";
    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation      = odometry_.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
}
