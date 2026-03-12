#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class MyStudentNode : public rclcpp::Node
{
public:
  MyStudentNode() : Node("kuzelna_studentska_node")
  {
    this->declare_parameter<double>("max_voltage", 42.0);
    this->declare_parameter<double>("min_voltage", 32.0);

    double max_v = this->get_parameter("max_voltage").as_double();
    double min_v = this->get_parameter("min_voltage").as_double();

    // KONTROLNÍ VÝPIS PŘI STARTU
    RCLCPP_INFO(this->get_logger(), "Node spustena! Parametry: min = %.1f V, max = %.1f V", min_v, max_v);

    name_publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MyStudentNode::timer_callback, this));

    battery_publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
    battery_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 10, std::bind(&MyStudentNode::battery_callback, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = this->get_name(); 
    name_publisher_->publish(message);
    
    // KONTROLNÍ VÝPIS
    RCLCPP_INFO(this->get_logger(), "cekam na napeti...");
  }

  void battery_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float voltage = msg->data;
    double max_v = this->get_parameter("max_voltage").as_double();
    double min_v = this->get_parameter("min_voltage").as_double();

    float percentage = (voltage - min_v) / (max_v - min_v) * 100.0f;

    auto out_msg = std_msgs::msg::Float32();
    out_msg.data = percentage;
    battery_publisher_->publish(out_msg);
    
    // VÝPIS PŘEPOČTU BATERIE
    RCLCPP_INFO(this->get_logger(), "BATERIE: %.2f V -> %.2f %%", voltage, percentage);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr name_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyStudentNode>());
  rclcpp::shutdown();
  return 0;
}
