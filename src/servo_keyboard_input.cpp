#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

class ServoKeyboardInput : public rclcpp::Node
{
public:
  ServoKeyboardInput() : Node("servo_keyboard_input")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    RCLCPP_INFO(this->get_logger(), "矢印キーとW/Sで方向指定：←(L) →(R) ↑(U) ↓(D) W=F（前） S=B（後）");

    timer_ = this->create_wall_timer(
      100ms, std::bind(&ServoKeyboardInput::check_key_input, this));
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void check_key_input()
  {
    int key = getch_nonblock();
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    const double speed = 0.3;
    bool send = false;

    if (key == 27) {
      if (getch_nonblock() == 91) {
        int arrow = getch_nonblock();
        switch (arrow) {
          case 68: msg.twist.linear.x = -speed; RCLCPP_INFO(this->get_logger(), "← L"); send = true; break;
          case 67: msg.twist.linear.x =  speed; RCLCPP_INFO(this->get_logger(), "→ R"); send = true; break;
          case 65: msg.twist.linear.z =  speed; RCLCPP_INFO(this->get_logger(), "↑ U"); send = true; break;
          case 66: msg.twist.linear.z = -speed; RCLCPP_INFO(this->get_logger(), "↓ D"); send = true; break;
        }
      }
    } else if (key == 'w' || key == 'W') {
      msg.twist.linear.y = speed;
      RCLCPP_INFO(this->get_logger(), "W: F（前進）");
      send = true;
    } else if (key == 's' || key == 'S') {
      msg.twist.linear.y = -speed;
      RCLCPP_INFO(this->get_logger(), "S: B（後退）");
      send = true;
    }

    if (send) {
      publisher_->publish(msg);

      // 停止メッセージを少し後に送信
      rclcpp::sleep_for(100ms);
      auto stop_msg = msg;
      stop_msg.twist.linear.x = 0.0;
      stop_msg.twist.linear.y = 0.0;
      stop_msg.twist.linear.z = 0.0;
      publisher_->publish(stop_msg);
    }
  }

  int getch_nonblock()
  {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return (ch != EOF) ? ch : -1;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoKeyboardInput>());
  rclcpp::shutdown();
  return 0;
}
