#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MyStudentNode : public rclcpp::Node
{
public:
  // Tady nastavujeme název naší Node
  MyStudentNode() : Node("kuzelna_studentska_node")
  {
    // Vytvoření publisheru na topic "node_name"
    publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    // Timer, který každých 500ms zavolá funkci timer_callback
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MyStudentNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    // Do zprávy vložíme název naší Node pomocí metody get_name()
    message.data = this->get_name(); 
    RCLCPP_INFO(this->get_logger(), "Publikuji svuj nazev: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyStudentNode>());
  rclcpp::shutdown();
  return 0;
}
