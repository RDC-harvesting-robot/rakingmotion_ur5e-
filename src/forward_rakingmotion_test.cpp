#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <executor_msgs/srv/plan.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <optional>
#include <mutex>
#include <future>

using namespace std::chrono_literals;
using Plan = executor_msgs::srv::Plan;

class ServoForwardNode : public rclcpp::Node


{
public:
  ServoForwardNode() : Node("servo_forward_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
                       state_(State::STOPPED) // 初期状態は停止
  {

    // --- パラメータ宣言 ---
    base_frame_     = this->declare_parameter<std::string>("base_frame", "base_link");
    tool_frame_     = this->declare_parameter<std::string>("tool_frame", "tool0");
    twist_topic_    = this->declare_parameter<std::string>("twist_topic", "/servo_node/delta_twist_cmds");
    force_topic_    = this->declare_parameter<std::string>("force_topic", "/calibrated_force_data");
    std::string direction = this->declare_parameter<std::string>("direction", "R"); // 前進方向 (R:+X, L:-X)
    linear_x_sign_  = (direction == "R") ? 1.0 : -1.0;
    max_distance_   = this->declare_parameter<double>("max_distance", 0.2); // 前進距離
    force_limit_xy_ = this->declare_parameter<double>("force_limit_xy", 6.0); // 力制限
    max_speed_      = this->declare_parameter<double>("max_speed", 0.4); // 最大速度

    // (★追加) 横スライド用パラメータ
    slide_distance_ = this->declare_parameter<double>("slide_distance", 0.1); // スライド距離 (m)
    slide_direction_sign_ = this->declare_parameter<double>("slide_direction", 1.0); // スライド方向 (+1.0: +Y, -1.0: -Y)
    slide_speed_    = this->declare_parameter<double>("slide_speed", 0.1); // スライド速度 (m/s)

    // --- ROSインターフェース ---
    cg_srv_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cg_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, rclcpp::QoS(20).reliable());
    pub_dist_  = this->create_publisher<geometry_msgs::msg::PointStamped>("/distance_from_start", 10); // 前進距離表示用
    sub_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      force_topic_, 10, std::bind(&ServoForwardNode::force_cb, this, std::placeholders::_1));

    rclcpp::QoS latched(1); latched.transient_local().reliable();
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/initial_pose", latched); // 開始位置表示用

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/calculated_velocity", 10); // 速度推定 (おまけ)

    // MoveIt Servo 制御用クライアント
    start_servo_cli_   = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    unpause_servo_cli_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/unpause_servo");

    // サービスサーバー (/forward/plan を受け付ける)
    srv_ = this->create_service<Plan>(
      "/forward/plan",
      std::bind(&ServoForwardNode::handle_plan, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      cg_srv_);

    // 制御ループタイマー
    timer_ = this->create_wall_timer(5ms, std::bind(&ServoForwardNode::on_timer, this), cg_timer_);

    RCLCPP_INFO(get_logger(), "[ForwardSlide] ready base=%s tool=%s dir=%s topic=%s",
                base_frame_.c_str(), tool_frame_.c_str(), direction.c_str(), twist_topic_.c_str());
  }

private:
  // (★修正) ステートマシンに SLIDING_SIDEWAYS を追加
  enum class State { MOVING_FORWARD, WAITING_H, MOVING_HORIZONAL,STOPPED };
  State state_;

  // サービスハンドラ: サービスが呼ばれたら状態をリセットして開始
  void handle_plan(const std::shared_ptr<Plan::Request> req, std::shared_ptr<Plan::Response> res)
  {
    (void)req;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (started_) {
        RCLCPP_WARN(get_logger(), "[ForwardSlide] busy -> reject");
        res->result = "busy";
        return;
      }
      // Start servo
      if (start_servo_cli_->wait_for_service(500ms))
        (void)start_servo_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      if (unpause_servo_cli_->wait_for_service(500ms))
        (void)unpause_servo_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

      started_ = true;
      initialized_ = false;
      published_init_ = false;
      velocity_initialized_ = false;
      state_ = State::MOVING_FORWARD; // (★修正) 状態を前進にセット
      done_promise_.emplace();
      done_future_ = done_promise_->get_future();
    }

    // Wait until motion complete (STOPPED 状態になるまで)
    auto status = done_future_.wait_for(std::chrono::minutes(10));
    res->result = (status == std::future_status::ready) ? done_future_.get() : "timeout";
    RCLCPP_INFO(get_logger(), "[ForwardSlide] service done: result=\"%s\"", res->result.c_str());
  }

  // ==== Timer loop (★修正) ====
  void on_timer()
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
    double abs_force=std::sqrt((fx*fx)+(fy*fy));


    if (!initialized_) {
      initial_x_ = x;
      initial_y_ = y;
      initial_z_ = z;
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "初期位置: (%.3f, %.3f, %.3f)", x, y, z);
    }

    // double dist = std::sqrt(
    //   std::pow(x - initial_x_, 2) +
    //   std::pow(y - initial_y_, 2) +
    //   std::pow(z - initial_z_, 2));
    double x_dist = x - initial_x_;
    double y_dist = y - initial_y_;
    publish_distance(x_dist,y_dist);  // 距離をパブリッシュ
    velocity = velocity_calculation(abs_force); 
    

    RCLCPP_INFO(this->get_logger(), "[状態: %d] 距離x: %.3f距離y: %.3f m, 速度: %.3f m/s,力: x_y=%.2f",
                static_cast<int>(state_), x_dist,y_dist,velocity,abs_force);

    // --- ステートマシンによる制御 ---
    switch (state_) {
      case State::MOVING_FORWARD:
        if (y_dist >= 0.2 || exceeded_force()) {
          RCLCPP_INFO(this->get_logger(), "前進停止 → 5秒待機");
          state_ = State::WAITING_H;
          x_dist = 0;
          stop_time_ = this->now();
          publish_stop();
        } else {
          publish_velocity(0,linear_y_*velocity);
        }
        break;

      case State::WAITING_H:
        if ((this->now() - stop_time_).seconds() >= 2.0) {
          RCLCPP_INFO(this->get_logger(), "3秒経過 →  左右に移動");
          state_ = State::MOVING_HORIZONAL;
        }
        break;

      case State::MOVING_HORIZONAL:
        if (x_dist >= 0.2 || exceeded_force()) {
          RCLCPP_INFO(this->get_logger(), "横進停止 → 5秒待機");
          state_ = State::STOPPED;
          stop_time_ = this->now();
          publish_stop();
        } else {
          publish_velocity(linear_x_*velocity,0);
        }
        break;

      case State::STOPPED:
        publish_stop(); // 念のため停止指令を送信

        // (★修正★) ノードを終了させる代わりに、サービスを完了させる
        {
          std::lock_guard<std::mutex> lk(mtx_);
          if (started_) { // まだ完了通知を送っていなければ
              started_ = false; // タイマーの активный 処理を停止
              if (done_promise_) {
                  done_promise_->set_value("motion_complete"); // サービスに完了を通知
                  done_promise_.reset(); // promiseをリセット
              }
              RCLCPP_INFO(this->get_logger(), "モーション完了。待機状態に戻ります。");
          }
        }
        // rclcpp::shutdown(); // ←★削除★
        return;
    }


    // --- 速度推定ロジック (おまけ、状態とは独立) ---
    rclcpp::Time now_time = this->get_clock()->now();
    if (velocity_initialized_) {
      double dt = (now_time - last_time_).seconds();
      if (dt >= 0.0001 && dt <= 0.1) {
        double vx = (x - last_x_) / dt;
        double vy = (y - last_y_) / dt;
        double vz = (z - last_z_) / dt;

        if (( abs_force < force_limit_xy_ && (vx != 0.0 || vy != 0.0 || vz != 0.0)) ||
             abs_force > force_limit_xy_) {
          auto vel_msg = geometry_msgs::msg::TwistStamped();
          vel_msg.header.stamp = now_time;
          vel_msg.header.frame_id = base_frame_;
          vel_msg.twist.linear.x = vx;
          vel_msg.twist.linear.y = vy;
          vel_msg.twist.linear.z = vz;
          velocity_pub_->publish(vel_msg);
          last_time_ = now_time;
          last_x_ = x;
          last_y_ = y;
          last_z_ = z;
        }
      }
    } else {
      last_time_ = now_time;
      last_x_ = x;
      last_y_ = y;
      last_z_ = z;
      velocity_initialized_ = true;
    }
  } // on_timer の終わり

  // 力覚センサのコールバック
  void force_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  { fx = msg->wrench.force.x; fy = msg->wrench.force.y; fz = msg->wrench.force.z; }

  // (★修正) 速度指令パブリッシュ関数 (XとYを受け取る)
  void publish_velocity(double vx, double vy)
  {
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now();
    m.header.frame_id = base_frame_;
    m.twist.linear.x = vx;
    m.twist.linear.y = vy; // Y軸速度も設定
    m.twist.linear.z = 0.0;
    m.twist.angular.x = 0.0;
    m.twist.angular.y = 0.0;
    m.twist.angular.z = 0.0;
    pub_twist_->publish(m);
  }

  // 距離パブリッシュ関数
  void publish_distance(double dx,double dy)
  {
    geometry_msgs::msg::PointStamped p;
    p.header.stamp = now(); p.header.frame_id = base_frame_;
    p.point.x = dx; // 距離をX座標としてパブリッシュ
    p.point.y = dy;
    p.point.z = 0.0;
    pub_dist_->publish(p);
  }

  // 力に応じた速度計算関数
  double velocity_calculation(double force)
  {
    if(force >= force_limit_xy_) force = force_limit_xy_;
    double velocity = max_speed_ * (1.0 - (force / force_limit_xy_));
    if(velocity < 0.0) velocity = 0.0;
    return velocity;
  }

  // 停止指令関数
  void publish_stop()
  {
    publish_velocity(0.0, 0.0); // XとY両方を0にする
  }

  // 力制限チェック関数
  bool exceeded_force()
  {
    return std::sqrt(fx*fx + fy*fy) >= force_limit_xy_;
  }

  // --- メンバ変数 ---
  // ROSインターフェース
  std::string base_frame_, tool_frame_, twist_topic_, force_topic_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  pub_dist_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_force_;
  rclcpp::Service<Plan>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr cg_srv_, cg_timer_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_cli_, unpause_servo_cli_;
  rclcpp::Time stop_time_;

  double linear_x_  =0.,linear_y_=0.;

  // 速度推定用
  bool velocity_initialized_ = false;
  double last_x_ = 0.0, last_y_ = 0.0, last_z_ = 0.0;
  rclcpp::Time last_time_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // パラメータ
  double linear_x_sign_{1.0}, max_distance_{0.2}, force_limit_xy_{6.0}, max_speed_{0.4};
  double slide_distance_{0.1}, slide_direction_sign_{1.0}, slide_speed_{0.1}; // (★追加)

  // 状態変数
  double fx{0}, fy{0}, fz{0};
  double initial_x_, initial_y_, initial_z_;
  double slide_start_y_; // (★追加) スライド開始時のY座標

  // 実行フラグ
  bool started_{false}, initialized_{false}, published_init_{false};

  // 完了通知用
  std::mutex mtx_;
  std::optional<std::promise<std::string>> done_promise_{};
  std::future<std::string> done_future_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoForwardNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}