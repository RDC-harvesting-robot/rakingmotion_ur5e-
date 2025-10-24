#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp> // WrenchStamped のインクルード
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h> // (修正) .h を削除
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class MoveAfterCollecting : public rclcpp::Node {
public:
  MoveAfterCollecting()
  : Node("move_after_detected_leaf"),
    current_step_(Step::WAIT_FOR_INPUT),
    wait_started_(false),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    moving_(false),
    received_point_(false)
  {
    // (修正) geometry_msgs::msg::PointStamped に変更
    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/center_point", // クリック点 (A)
        rclcpp::SensorDataQoS(),
        std::bind(&MoveAfterCollecting::point_callback, this, std::placeholders::_1));

    // (修正) geometry_msgs::msg::PointStamped に変更
    leaf_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/click_point", // 重心点 (B)
      rclcpp::SensorDataQoS(),
      std::bind(&MoveAfterCollecting::leaf_point_callback, this, std::placeholders::_1));

    // (修正) geometry_msgs::msg::TwistStamped に変更
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    // (修正) geometry_msgs::msg::WrenchStamped に変更
    force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/calibrated_force_data", 10,
      std::bind(&MoveAfterCollecting::force_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      50ms, std::bind(&MoveAfterCollecting::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "/center_point と /click_point を待機中...");
  }

// --- 以下、変更なし ---
private:
  enum class Step { WAIT_FOR_INPUT, X, Y, Z, R_X, R_R, WAIT, R_Y, DONE };
  Step current_step_;

  rclcpp::Time wait_start_time_;
  bool wait_started_ = false;

  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (received_point_) return;

    edge_point_ = msg->point;
    has_edge_point_ = true;
    RCLCPP_INFO(this->get_logger(), "クリック点(A) 受信: (%.3f, %.3f, %.3f)", msg->point.x, msg->point.y, msg->point.z);

    if (has_leaf_point_) {
      calculate_target_and_start_moving();
    }
  }

  void leaf_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (received_point_) return;

    leaf_point_ = msg->point;
    has_leaf_point_ = true;
    RCLCPP_INFO(this->get_logger(), "重心点(B) 受信: (%.3f, %.3f, %.3f)", msg->point.x, msg->point.y, msg->point.z);

    if (has_edge_point_) {
      calculate_target_and_start_moving();
    }
  }

  void calculate_target_and_start_moving() {
    if (received_point_) return;

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("base_link", "tool0", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "初期位置取得に失敗: %s", ex.what());
      has_edge_point_ = false;
      has_leaf_point_ = false;
      return;
    }

    double initial_x = transform.transform.translation.x;
    double initial_y = transform.transform.translation.y;
    double initial_z = transform.transform.translation.z;

    const auto & ptA = edge_point_;

    target_x_ = initial_x + ptA.x;
    target_y_ = initial_y + ptA.y;
    target_z_ = initial_z + ptA.z;

    received_point_ = true;
    moving_ = true;
    current_step_ = Step::X;

    RCLCPP_INFO(this->get_logger(),
      "目標座標受信（相対移動）: 初期(%.3f, %.3f, %.3f) + 相対(A: %.3f, %.3f, %.3f) → 目標(%.3f, %.3f, %.3f)",
      initial_x, initial_y, initial_z, ptA.x, ptA.y, ptA.z,
      target_x_, target_y_, target_z_);
  }


  void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    fx = msg->wrench.force.x;
    fy = msg->wrench.force.y;
    fz = msg->wrench.force.z;
  }


  void control_loop() {
    if (!moving_ || current_step_ == Step::DONE || current_step_ == Step::WAIT_FOR_INPUT) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("base_link", "tool0", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF取得失敗: %s", ex.what());
      return;
    }

    double current_x = transform.transform.translation.x;
    double current_y = transform.transform.translation.y;
    double current_z = transform.transform.translation.z;

    double dx = (target_x_-0.04) - current_x;
    double dy = (target_y_-0.1) - current_y;
    double dz = (target_z_+0.08) - current_z;

    double abs_force=std::abs(fx)+std::abs(fy)+std::abs(fz);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
       "Step: %d, 誤差: dx=%.3f, dy=%.3f, dz=%.3f, force:fx=%.3f,fy=%.3f,fz=%.3f ",
       static_cast<int>(current_step_), dx, dy, dz,fx,fy,fz);

    double threshold = 0.01;
    double scale = velocity_calculation(abs_force);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->get_clock()->now();
    twist.header.frame_id = "base_link";

    switch (current_step_) {
      case Step::WAIT_FOR_INPUT:
        return;

      case Step::X:
        if (std::abs(dx) < threshold) {
          current_step_ = Step::Z;
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "X到達。次はZへ。");
          return;
        }
        twist.twist.linear.x = scale * (dx > 0 ? 1 : -1);
        break;

      case Step::Z:
      if (std::abs(dz) < threshold) {
        current_step_ = Step::Y;
        publish_stop();
        RCLCPP_INFO(this->get_logger(), "Z到達。次はYへ。");
        return;
      }
      twist.twist.linear.z = scale * (dz > 0 ? 1 : -1);
      break;

      case Step::Y:
        if (std::abs(dy) < threshold) {
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "Y到達。次はかき集め(R_X)へ。");

          if (!calculate_raking_direction()) {
            RCLCPP_ERROR(this->get_logger(), "かき集め方向を計算できません。停止します。");
            current_step_ = Step::DONE;
            rclcpp::shutdown();
            return;
          }
          raking_start_x_ = current_x;
          raking_start_y_ = current_y;

          current_step_ = Step::R_X;
          return;
        }
        twist.twist.linear.y = scale * (dy > 0 ? 1 : -1);
        break;

      case Step::R_X:
      {
        double dist_traveled = std::sqrt(std::pow(current_x - raking_start_x_, 2) +
                                         std::pow(current_y - raking_start_y_, 2));

        if (dist_traveled >= 0.2 || abs_force >= 6.0) {
          current_step_ = Step::WAIT;
          wait_start_time_ = this->get_clock()->now();
          wait_started_ = true;
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "raking motion 停止。5秒待機。");
          return;
        }
        twist.twist.linear.x = scale * raking_dir_x_;
        twist.twist.linear.y = scale * raking_dir_y_;
      }
      break;

     case Step::WAIT:
      if (wait_started_) {
        rclcpp::Duration elapsed = this->get_clock()->now() - wait_start_time_;
        if (elapsed.seconds() >= 5.0) {
          RCLCPP_INFO(this->get_logger(), "5秒待機完了 → 戻り(R_R)");
          current_step_ = Step::R_R;
        } else {
          publish_stop();
        }
      }
      break;

      case Step::R_R:
      {
        double return_dx = raking_start_x_ - current_x;
        double return_dy = raking_start_y_ - current_y;
        double dist_to_start = std::sqrt(return_dx*return_dx + return_dy*return_dy);

        if (dist_to_start <= threshold) {
          current_step_ = Step::R_Y;
          target_y_ = (current_y - 0.1);
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "raking motion return 完了。Y軸後退へ。");
          return;
        }

        double return_mag = dist_to_start;
        // ゼロ除算を回避
        double return_dir_x = (return_mag > 1e-6) ? (return_dx / return_mag) : 0.0;
        double return_dir_y = (return_mag > 1e-6) ? (return_dy / return_mag) : 0.0;

        twist.twist.linear.x = scale * return_dir_x;
        twist.twist.linear.y = scale * return_dir_y;
      }
      break;

      case Step::R_Y:
      {
        double return_dy_final = target_y_ - current_y;

        if (std::abs(return_dy_final) < threshold) {
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "Y軸後退完了。シャットダウン。");
          current_step_ = Step::DONE;
          rclcpp::shutdown();
          return;
        }
        twist.twist.linear.y = scale * (return_dy_final > 0 ? 1 : -1);
      }
      break;

      case Step::DONE:
        publish_stop();
        return;
    }

    twist_pub_->publish(twist);
  }

  void publish_stop() {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->get_clock()->now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = 0.0;
    twist.twist.linear.y = 0.0;
    twist.twist.linear.z = 0.0;
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;
    twist_pub_->publish(twist);
  }

  bool exceeded_force()
  {
    return (std::abs(fx) + std::abs(fy)) > 6.0 ;
  }

  double velocity_calculation(double force)
  {
    if(force>=6.0)force=6.0;
    double velocity=1.0*(1.0-((1.0/6.0))*force);
    if(velocity >= 1.0) velocity=1.0;
    return velocity;
  }

  bool calculate_raking_direction() {
    if (!has_edge_point_ || !has_leaf_point_) {
      RCLCPP_WARN(this->get_logger(), "ポイント未取得: A=%d, B=%d", has_edge_point_, has_leaf_point_);
      return false;
    }

    double dx = leaf_point_.x - edge_point_.x;
    double dy = leaf_point_.y - edge_point_.y;

    double mag = std::sqrt(dx*dx + dy*dy);

    if (mag < 0.001) {
      RCLCPP_WARN(this->get_logger(), "クリック点と重心点がほぼ同じです。X方向(+1)に移動します。");
      raking_dir_x_ = 1.0;
      raking_dir_y_ = 0.0;
      return true;
    }

    raking_dir_x_ = dx / mag;
    raking_dir_y_ = dy / mag;

    RCLCPP_INFO(this->get_logger(), "かき集め方向(A->B)を計算 (X: %.3f, Y: %.3f)", raking_dir_x_, raking_dir_y_);
    return true;
  }


  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr leaf_point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::Point edge_point_;
  geometry_msgs::msg::Point leaf_point_;

  double target_x_, target_y_, target_z_;
  bool moving_;
  bool received_point_;
  double fx = 0, fy = 0, fz = 0;
  bool has_edge_point_ = false;
  bool has_leaf_point_ = false;

  double raking_dir_x_ = 0.0;
  double raking_dir_y_ = 0.0;
  double raking_start_x_ = 0.0;
  double raking_start_y_ = 0.0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveAfterCollecting>());
  rclcpp::shutdown();
  return 0;
}