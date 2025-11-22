#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp> // WrenchStamped のインクルード
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
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
    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/center_point", // クリック点 (A)
        rclcpp::SensorDataQoS(),
        std::bind(&MoveAfterCollecting::point_callback, this, std::placeholders::_1));

    leaf_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/click_point", // 重心点 (B)
      rclcpp::SensorDataQoS(),
      std::bind(&MoveAfterCollecting::leaf_point_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/calibrated_force_data", 10,
      std::bind(&MoveAfterCollecting::force_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      50ms, std::bind(&MoveAfterCollecting::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "/center_point と /click_point を待機中...");
  }

private:
  enum class Step { WAIT_FOR_INPUT, X, Y, Z, R_X, R_R, WAIT, R_Y,R_S, DONE };
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

    initial_x = transform.transform.translation.x;
    initial_y = transform.transform.translation.y;
    initial_z = transform.transform.translation.z;

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

    // アプローチ目標への誤差
    double dx_approach = (target_x_ - 0.04) - current_x;
    double dy_approach = (target_y_ - 0.1) - current_y;
    double dz_approach = (target_z_ + 0.08) - current_z;

    // (★修正) 力の計算: XZ平面なので fx と fz を使用 (fy は無視)
    double abs_force = std::abs(fx) + std::abs(fz) + std::abs(fy);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
       "Step: %d, Approach Err: dx=%.3f, dy=%.3f, dz=%.3f, Force(XZ):%.3f (fx=%.2f, fz=%.2f)",
       static_cast<int>(current_step_), dx_approach, dy_approach, dz_approach, abs_force, fx, fz);

    double threshold = 0.01;
    double scale = velocity_calculation(abs_force);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->get_clock()->now();
    twist.header.frame_id = "base_link";

    switch (current_step_) {
      case Step::WAIT_FOR_INPUT:
        return;

      // --- アプローチ (A点基準) ---
      case Step::X:
        if (std::abs(dx_approach) < threshold) {
          current_step_ = Step::Z;
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "X到達。次はZへ。");
          return;
        }
        twist.twist.linear.x = scale * (dx_approach > 0 ? 1 : -1);
        break;

      case Step::Z:
      if (std::abs(dz_approach) < threshold) {
        current_step_ = Step::Y;
        publish_stop();
        RCLCPP_INFO(this->get_logger(), "Z到達。次はYへ。");
        return;
      }
      twist.twist.linear.z = scale * (dz_approach > 0 ? 1 : -1);
      break;

      case Step::Y:
        if (std::abs(dy_approach) < threshold) {
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "Y到達。次はかき集め(R_X)へ。");

          // (★修正) XZ方向ベクトルを計算
          if (!calculate_raking_direction()) {
            RCLCPP_ERROR(this->get_logger(), "かき集め方向を計算できません。停止します。");
            current_step_ = Step::DONE;
            rclcpp::shutdown();
            return;
          }
          // (★修正) 開始位置のXとZを保存
          raking_start_x_ = current_x;
          raking_start_z_ = current_z; // YではなくZ

          current_step_ = Step::R_X;
          return;
        }
        twist.twist.linear.y = scale * (dy_approach > 0 ? 1 : -1);
        break;

      // --- かき集め (A->B方向へXZ移動) ---
      case Step::R_X:
      {
        // (★修正) 停止条件: 開始地点からのXZ平面距離
        double dist_traveled = std::sqrt(std::pow(current_x - raking_start_x_, 2) +
                                         std::pow(current_z - raking_start_z_, 2)); // YではなくZ

        if (dist_traveled >= 0.2 || abs_force >= 6.0) { // 20cm移動 or 力
          current_step_ = Step::WAIT;
          wait_start_time_ = this->get_clock()->now();
          wait_started_ = true;
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "raking motion 停止。5秒待機。");
          return;
        }
        // (★修正) XZ方向に移動
        twist.twist.linear.x = scale * raking_dir_x_;
        twist.twist.linear.z = scale * raking_dir_z_; // YではなくZ
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

      // --- 戻り (かき集め開始地点へXZ移動) ---
      case Step::R_R:
      {
        // (★修正) 戻り目標: かき集め開始地点 (X, Z)
        double return_dx = raking_start_x_ - current_x;
        double return_dz = raking_start_z_ - current_z; // YではなくZ
        double dist_to_start = std::sqrt(return_dx*return_dx + return_dz*return_dz); // YではなくZ

        if (dist_to_start <= threshold) {
          current_step_ = Step::R_Y;
          target_y_ = (current_y - 0.1); // 最終後退のY目標を設定
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "raking motion return 完了。Y軸後退へ。");
          return;
        }

        double return_mag = dist_to_start;
        // ゼロ除算を回避
        double return_dir_x = (return_mag > 1e-6) ? (return_dx / return_mag) : 0.0;
        double return_dir_z = (return_mag > 1e-6) ? (return_dz / return_mag) : 0.0; // YではなくZ

        // (★修正) XZ方向に移動
        twist.twist.linear.x = scale * return_dir_x;
        twist.twist.linear.z = scale * return_dir_z; // YではなくZ
      }
      break;

      // --- 後退 (Y軸) ---
      case Step::R_Y:
      {
        double return_dy_final = target_y_ - current_y;

        if (std::abs(return_dy_final) < threshold) {
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "Y軸後退完了.初期位置へ");
          current_step_ = Step::R_S;
          return;
        }
        twist.twist.linear.y = scale * (return_dy_final > 0 ? 1 : -1);
      }
      break;

      case Step::R_S:
      {
        // 初期位置への移動
        double return_dx = initial_x - current_x;
        double return_dy = initial_y - current_y;
        double return_dz = initial_z - current_z;
        double dist_to_start = std::sqrt(return_dx*return_dx + return_dz*return_dz + return_dy*return_dy);
        if (dist_to_start < threshold) {
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "初期位置へ移動完了.動作を終了します");
          current_step_ = Step::DONE;
          rclcpp::shutdown();
          return;
        }
        double return_mag = dist_to_start;
        double return_dir_x = (return_mag > 1e-6) ? (return_dx / return_mag) : 0.0;
        double return_dir_y = (return_mag > 1e-6) ? (return_dy / return_mag) : 0.0;
        double return_dir_z = (return_mag > 1e-6) ? (return_dz / return_mag) : 0.0;

        twist.twist.linear.x = scale * return_dir_x;
        twist.twist.linear.y = scale * return_dir_y;
        twist.twist.linear.z = scale * return_dir_z;

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

  // (★修正) 力の閾値: XZ平面なので fx と fz を使用
  bool exceeded_force()
  {
    return (std::abs(fx) + std::abs(fz)) > 6.0 ;
  }

  // (★修正) 速度計算: 力は abs_force (XZ平面) を使う
  double velocity_calculation(double force_xz)
  {
    if(force_xz >= 6.0) force_xz = 6.0;
    // 1.0 m/s を最大速度とする (元のロジック)
    double velocity = 1.0 * (1.0 - (force_xz / 6.0));
    if(velocity < 0.0) velocity = 0.0; // 念のため
    // if(velocity >= 1.0) velocity=1.0; // このチェックは不要
    return velocity;
  }

  // (★修正) かき集め方向(A->B)のXZ正規化ベクトルを計算する
  bool calculate_raking_direction() {
    if (!has_edge_point_ || !has_leaf_point_) {
      RCLCPP_WARN(this->get_logger(), "ポイント未取得: A=%d, B=%d", has_edge_point_, has_leaf_point_);
      return false;
    }

    // A(edge_point_) と B(leaf_point_) のカメラ相対座標を使用
    double dx = leaf_point_.x - edge_point_.x; // B.x - A.x
    double dz = leaf_point_.z - edge_point_.z; // B.z - A.z (★YではなくZ)

    double mag = std::sqrt(dx*dx + dz*dz); // (★YではなくZ)

    if (mag < 0.001) {
      RCLCPP_WARN(this->get_logger(), "クリック点と重心点がほぼ同じです。X方向(+1)に移動します。");
      raking_dir_x_ = 1.0;
      raking_dir_z_ = 0.0; // (★YではなくZ)
      return true;
    }

    raking_dir_x_ = dx / mag;
    raking_dir_z_ = dz / mag; // (★YではなくZ)

    RCLCPP_INFO(this->get_logger(), "かき集め方向(A->B)を計算 (X: %.3f, Z: %.3f)", raking_dir_x_, raking_dir_z_); // (★YではなくZ)
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

  // (★修正) かき集め用変数を XZ に
  double raking_dir_x_ = 0.0;
  double raking_dir_z_ = 0.0; // YではなくZ
  double raking_start_x_ = 0.0;
  double raking_start_z_ = 0.0; // YではなくZ

  // 初期位置を記録しておく
  double initial_x;
  double initial_y;
  double initial_z;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveAfterCollecting>());
  rclcpp::shutdown();
  return 0;
}