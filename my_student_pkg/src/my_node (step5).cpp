#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// 1. Přidali jsme knihovnu pro typ Float32
#include "std_msgs/msg/float32.hpp" 

using namespace std::chrono_literals;

class MyStudentNode : public rclcpp::Node
{
public:
  MyStudentNode() : Node("kuzelna_studentska_node")
  {
    // Původní publisher pro jméno (Úkol 4)
    name_publisher_ = this->create_publisher<std_msgs::msg::String>("node_name", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MyStudentNode::timer_callback, this));

    // 2. Nový publisher pro procenta baterie (Úkol 5)
    battery_publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);

    // 3. Nový subscriber pro čtení napětí (Úkol 5)
    battery_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 10, std::bind(&MyStudentNode::battery_callback, this, std::placeholders::_1));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = this->get_name(); 
    name_publisher_->publish(message);
  }

  // 4. Tato funkce se zavolá POKAŽDÉ, když přijde nová zpráva o napětí baterie
  void battery_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float voltage = msg->data;
    
    // Výpočet procent: (aktuální - min) / (max - min) * 100
    float percentage = (voltage - 32.0f) / (42.0f - 32.0f) * 100.0f;

    // Vytvoření zprávy a její odeslání do světa
    auto out_msg = std_msgs::msg::Float32();
    out_msg.data = percentage;
    battery_publisher_->publish(out_msg);
    
    // Výpis do terminálu, abychom viděli, že to něco dělá
    RCLCPP_INFO(this->get_logger(), "Prijato napeti: %.2f V -> Odesilam: %.2f %%", voltage, percentage);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr name_publisher_;
  
  // Proměnné pro naši baterii
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
