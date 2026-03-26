#include "Localization.hpp"
#include "mpc_rbt_simulator/RobotConfig.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // 1. Inicializace zprávy odometrie
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";

    // 2. Vytvoření Publisheru pro odometrii (jména přesně podle tvého .hpp)
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

    // 3. Vytvoření Subscriberu pro čtení rychlosti kol (jména přesně podle tvého .hpp)
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));

    // 4. Inicializace TF broadcasteru pro odesílání transformací
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // Zjištění času, který uběhl od posledního měření (dt)
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // msg.velocity[0] je obvykle levé kolo, msg.velocity[1] pravé kolo
    updateOdometry(msg.velocity[1], msg.velocity[0], dt);
    // Časové razítko musí odpovídat aktuálnímu času výpočtu
    odometry_.header.stamp = current_time;

    publishOdometry();
    publishTransform();
}



void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // --- 1. Přímá kinematika diferenciálního podvozku ---
    double v_left = left_wheel_vel * robot_config::WHEEL_RADIUS;
    double v_right = right_wheel_vel * robot_config::WHEEL_RADIUS;
    
    double linear = (v_right + v_left) / 2.0;
    double angular = (v_right - v_left) / (2.0 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS);

    // --- 2. Získání aktuálního natočení (theta) robota z Quaternionů ---
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta); 

    // --- 3. Eulerova integrace rychlostí pro výpočet nové polohy (X, Y) ---
    double delta_x = linear * std::cos(theta) * dt;
    double delta_y = linear * std::sin(theta) * dt;
    double delta_theta = angular * dt;

    odometry_.pose.pose.position.x += delta_x;
    odometry_.pose.pose.position.y += delta_y;
    theta += delta_theta;

    // Normalizace úhlu, aby byl vždy mezi -PI a PI
    theta = std::atan2(std::sin(theta), std::cos(theta));

    // --- 4. Zabalení výsledků zpět do zprávy ---
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);

    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;
}

void LocalizationNode::publishOdometry() {
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = odometry_.header.stamp;
    t.header.frame_id = odometry_.header.frame_id; // "map"
    t.child_frame_id = odometry_.child_frame_id;   // "base_link"

    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0; 
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}
