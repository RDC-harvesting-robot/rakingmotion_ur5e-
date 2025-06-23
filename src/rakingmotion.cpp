#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

class ServoTwistMonitor : public rclcpp::Node
{
public:
  ServoTwistMonitor(const rclcpp::NodeOptions & options)
  : Node("servo_twist_monitor", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    state_(State::MOVING_FORWARD),
    initialized_(false)
  {
    std::string direction = this->declare_parameter<std::string>("direction", "R");
    if (direction == "R") {
      linear_x_ = 1;
    } else if (direction == "L") {
      linear_x_ = -1;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid direction: '%s'. Use 'L' or 'R'.", direction.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "開始: linear.x = %.2f", linear_x_);

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    distance_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/distance_from_start", 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&ServoTwistMonitor::timer_callback, this));

    force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/calibrated_force_data", 10,
      std::bind(&ServoTwistMonitor::force_callback, this, std::placeholders::_1));
  }

private:
  enum class State { MOVING_FORWARD, WAITING, MOVING_BACK, STOPPED };
  State state_;

  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("base_link", "tool0", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    double z = transform.transform.translation.z;

    double velocity=0;
    double abs_force=std::abs(fx)+std::abs(fy);


    if (!initialized_) {
      initial_x_ = x;
      initial_y_ = y;
      initial_z_ = z;
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "初期位置: (%.3f, %.3f, %.3f)", x, y, z);
    }

    double dist = std::sqrt(
      std::pow(x - initial_x_, 2) +
      std::pow(y - initial_y_, 2) +
      std::pow(z - initial_z_, 2));

    publish_distance(dist);  // 距離をパブリッシュ
    velocity = velocity_calculation(abs_force); 
    

    RCLCPP_INFO(this->get_logger(), "[状態: %d] 距離: %.3f m, 速度: %.3f m/s,力: x=%.2f, y=%.2f, z=%.2f",
                static_cast<int>(state_), dist,velocity,fx, fy, fz);

              

    switch (state_) {
      case State::MOVING_FORWARD:
        if (dist >= 0.3 || exceeded_force()) {
          RCLCPP_INFO(this->get_logger(), "前進停止 → 5秒待機");
          state_ = State::WAITING;
          stop_time_ = this->now();
          publish_stop();
        } else {
          publish_velocity(linear_x_*velocity);
        }
        break;

      case State::WAITING:
        if ((this->now() - stop_time_).seconds() >= 3.0) {
          RCLCPP_INFO(this->get_logger(), "5秒経過 → 初期位置へ戻る");
          state_ = State::MOVING_BACK;
        }
        break;

      case State::MOVING_BACK:
        if (std::abs(x - initial_x_) < 0.02 &&
            std::abs(y - initial_y_) < 0.02 &&
            std::abs(z - initial_z_) < 0.02) {
          RCLCPP_INFO(this->get_logger(), "初期位置に戻った → 停止");
          publish_stop();
          state_ = State::STOPPED;
        } else {
          publish_velocity(-linear_x_*velocity);
        }
        break;

      case State::STOPPED:
        publish_stop();
        rclcpp::shutdown(); 
        break;
    }
  }

  void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    fx = msg->wrench.force.x;
    fy = msg->wrench.force.y;
    fz = msg->wrench.force.z;
  }

  bool exceeded_force()
  {
    return std::abs(fx) > 6.0 || std::abs(fy) > 6.0 || std::abs(fz) > 6.0;
  } 

  void publish_velocity(double vx)
  {
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = vx;
    publisher_->publish(msg);
  }

  void publish_stop()
  { 
    publish_velocity(0.0);
  }

  void publish_distance(double distance)
  {
    auto msg = geometry_msgs::msg::PointStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.point.x = distance;
    msg.point.y = 0.0;
    msg.point.z = 0.0;
    distance_pub_->publish(msg);
  }

  double velocity_calculation(double force)
  {
    RCLCPP_INFO(this->get_logger(), "forece:%.3f",force);
    // double velocity=(150*(1-((1/6)*force)))/1000;
    double velocity=0.10*(1.0-((1.0/6.0))*force);
    if(velocity >= 0.10) velocity=0.10;
    return velocity;
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr distance_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double linear_x_;
  double fx = 0, fy = 0, fz = 0;
  double initial_x_, initial_y_, initial_z_;
  bool initialized_;
  rclcpp::Time stop_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoTwistMonitor>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
