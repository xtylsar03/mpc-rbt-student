#include <chrono>
#include <functional>
#include <termios.h>    // Pro nastavování terminálu
#include <unistd.h>     // Pro STDIN_FILENO a fuknci read()
#include <fcntl.h>      // Pro fcntl()
#include <sys/select.h> // Pro funkci select()

#include "KeyboardControl.hpp"

using namespace std::chrono_literals;

KeyboardControlNode::KeyboardControlNode(): rclcpp::Node("keyboard_control_node") {
    // 1. Z původního kódu: Vytvoření publisheru a timeru
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardControlNode::timerCallback, this));

    // NOVÉ: Deklarace parametru 'speed' s výchozí hodnotou 0.5
    this->declare_parameter<double>("speed", 0.5);

    // 2. Nastavení terminálu pro neblokující čtení kláves
    tcgetattr(STDIN_FILENO, &old_termios_);
    struct termios new_termios = old_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    RCLCPP_INFO(this->get_logger(), "Use Arrow Keys to control the robot. Press 'ctrl+c' to quit.");
    RCLCPP_INFO(this->get_logger(), "Dynamic speed parameter is active! Default: 0.5");
}

KeyboardControlNode::~KeyboardControlNode() {
    // Vrácení terminálu do původního stavu při vypnutí uzlu
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    geometry_msgs::msg::Twist twist{};
    char c;

    // NOVÉ: Načtení aktuální hodnoty parametru 'speed' (umožňuje dynamickou změnu za běhu)
    double current_speed = this->get_parameter("speed").as_double();

    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int retval = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

    if (retval > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\033') { // ESC sequence (arrow keys)
                char seq[2];
                if (read(STDIN_FILENO, &seq, 2) != 2)
                    return;

                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A':
                            twist.linear.x = current_speed;  // up arrow (dopředu)
                            break;
                        case 'B':
                            twist.linear.x = -current_speed; // down arrow (dozadu)
                            break;
                        case 'C':
                            twist.angular.z = -current_speed; // right arrow (doprava)
                            break;
                        case 'D':
                            twist.angular.z = current_speed;  // left arrow (doleva)
                            break;
                    }
                }
            }

            // Odeslání zprávy robotovi
            twist_publisher_->publish(twist);
        }
    }
}
