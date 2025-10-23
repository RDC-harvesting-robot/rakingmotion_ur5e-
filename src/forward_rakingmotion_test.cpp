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
  // (修正) state_ の初期値は STOPPED の方が安全
  ServoForwardNode() : Node("servo_forward_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), 
                       state_(State::STOPPED) 
  {
    
    // (修正) パラメータのデフォルト値を単腕用に変更
    base_frame_     = this->declare_parameter<std::string>("base_frame", "base_link");
    tool_frame_     = this->declare_parameter<std::string>("tool_frame", "tool0");
    twist_topic_    = this->declare_parameter<std::string>("twist_topic", "/servo_node/delta_twist_cmds");
    force_topic_    = this->declare_parameter<std::string>("force_topic", "/calibrated_force_data");
    
    std::string direction = this->declare_parameter<std::string>("direction", "R");
    linear_x_sign_  = (direction == "R") ? 1.0 : -1.0;
    max_distance_   = this->declare_parameter<double>("max_distance", 0.2);
    force_limit_xy_ = this->declare_parameter<double>("force_limit_xy", 6.0);
    max_speed_      = this->declare_parameter<double>("max_speed", 0.4); // m/s

   
    cg_srv_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cg_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);


    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, rclcpp::QoS(20).reliable());
    pub_dist_  = this->create_publisher<geometry_msgs::msg::PointStamped>("/distance_from_start", 10);
    sub_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      force_topic_, 10, std::bind(&ServoForwardNode::force_cb, this, std::placeholders::_1));

    rclcpp::QoS latched(1); latched.transient_local().reliable();
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/initial_pose", latched);

    // MoveIt Servo control (optional but helpful)
    start_servo_cli_   = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    unpause_servo_cli_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/unpause_servo");

    // Service (Humble signature)
    srv_ = this->create_service<Plan>(
      "/forward/plan",
      std::bind(&ServoForwardNode::handle_plan, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      cg_srv_);

    // Control loop
    timer_ = this->create_wall_timer(5ms, std::bind(&ServoForwardNode::on_timer, this), cg_timer_);

    RCLCPP_INFO(get_logger(), "[Forward] ready base=%s tool=%s dir=%s topic=%s",
                base_frame_.c_str(), tool_frame_.c_str(), direction.c_str(), twist_topic_.c_str());

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/calculated_velocity", 10);
    }

private:
  // (注) この state_ は現在のロジックでは使用されていませんが、定義は残します
  enum class State { MOVING_FORWARD, WAITING, MOVING_BACK, STOPPED };
  State state_;

  void handle_plan(const std::shared_ptr<Plan::Request> req, std::shared_ptr<Plan::Response> res)
  {
    (void)req;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (started_) {
        RCLCPP_WARN(get_logger(), "[Forward] busy -> reject");
        res->result = "busy";
        return;
      }
      // Start servo (best-effort)
      if (start_servo_cli_->wait_for_service(500ms))
        (void)start_servo_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      if (unpause_servo_cli_->wait_for_service(500ms))
        (void)unpause_servo_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

      started_ = true;
      initialized_ = false;
      published_init_ = false;
      velocity_initialized_ = false; // 速度推定もリセット
      done_promise_.emplace();
      done_future_ = done_promise_->get_future();
    }

    // Wait until motion complete
    auto status = done_future_.wait_for(std::chrono::minutes(10));
    res->result = (status == std::future_status::ready) ? done_future_.get() : "timeout";
    RCLCPP_INFO(get_logger(), "[Forward] service done: result=\"%s\"", res->result.c_str());
  }

  // ==== Timer loop ====
  void on_timer()
  {
    if (!started_) { publish_velocity(0.0); return; }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(base_frame_, tool_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF failed: %s", ex.what());
      publish_velocity(0.0); return;
    }

    const double x = tf.transform.translation.x;
    const double y = tf.transform.translation.y;
    const double z = tf.transform.translation.z;

    double velocity=0;
    double abs_force=std::sqrt((fx_*fx_)+(fy_*fy_));


    if (!initialized_) {
      initial_x_ = x;
      initial_y_ = y;
      initial_z_ = z;
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "初期位置: (%.3f, %.3f, %.3f)", x, y, z);
      geometry_msgs::msg::PointStamped init;
      init.header.stamp = now();
      init.header.frame_id = base_frame_;
      init.point.x = initial_x_;
      init.point.y = initial_y_;
     init.point.z = initial_z_;
      initial_pose_pub_->publish(init);
      RCLCPP_INFO(this->get_logger(), "[Forward] /initial_pose published");
    }

    double dist = std::sqrt(
      std::pow(x - initial_x_, 2) +
      std::pow(y - initial_y_, 2) +
      std::pow(z - initial_z_, 2));

    publish_distance(dist);  // 距離をパブリッシュ
    velocity = velocity_calculation(abs_force); 
    
    // (修正) ログの state_ は現在のロジックでは参照専用
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, // ログが多すぎないよう Throttle
                 "[状態: %d] 距離: %.3f m, 速度: %.3f m/s,力: x_y=%.2f",
                 static_cast<int>(state_), dist,velocity,abs_force);

              
    // (元のアルゴリズム) 前進して停止
    if (dist >= max_distance_ || exceeded_force()) {
        RCLCPP_INFO(this->get_logger(), "前進停止 (目標距離または力制限に到達)");
        publish_stop();
        {
          std::lock_guard<std::mutex> lk(mtx_);
          started_ = false;
          if (done_promise_) { done_promise_->set_value("raking_end"); done_promise_.reset(); }
        }
        return; // 停止したらタイマー処理終了
      } else {
        publish_velocity(linear_x_sign_*velocity);
      }
      
      
    // 速度推定ロジック
    rclcpp::Time now_time = this->get_clock()->now();
    if (velocity_initialized_) {
      double dt = (now_time - last_time_).seconds();
      if (dt >= 0.0001 && dt <= 0.1) {
        double vx = (x - last_x_) / dt;
        double vy = (y - last_y_) / dt;
        double vz = (z - last_z_) / dt;

        // (修正) 6 -> force_limit_xy_
        if (( abs_force < force_limit_xy_ && (vx != 0.0 || vy != 0.0 || vz != 0.0)) ||
             abs_force > force_limit_xy_) {
          auto vel_msg = geometry_msgs::msg::TwistStamped();
          vel_msg.header.stamp = now_time;
          // (修正) "base_link" -> base_frame_
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
  }

  void force_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  { fx_ = msg->wrench.force.x; fy_ = msg->wrench.force.y; fz_ = msg->wrench.force.z; }

  void publish_velocity(double vx)
  {
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now(); m.header.frame_id = base_frame_;
    m.twist.linear.x = vx;
    pub_twist_->publish(m);
  }
  void publish_distance(double d)
  {
    geometry_msgs::msg::PointStamped p;
    p.header.stamp = now(); p.header.frame_id = base_frame_;
    p.point.x = d; pub_dist_->publish(p);
  }

  // (修正) 1.0 m/s ではなく、max_speed_ を使用する
  double velocity_calculation(double force)
  {
    if(force >= force_limit_xy_) force = force_limit_xy_;

    // 0Nの時に max_speed_ (例: 0.4) になり、
    // force_limit_xy_ (例: 6.0N) の時に 0.0 になるように線形補間
    double velocity = max_speed_ * (1.0 - (force / force_limit_xy_));
    
    if(velocity < 0.0) velocity = 0.0;
    return velocity;
  }

  void publish_stop()
  { 
    publish_velocity(0.0);
  }
  bool exceeded_force()
  {
    return std::sqrt(fx_*fx_ + fy_*fy_) >= force_limit_xy_;
  } 

  // IO
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
  
  // 速度推定用
  bool velocity_initialized_ = false;
  double last_x_, last_y_, last_z_;
  rclcpp::Time last_time_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // params/state
  double linear_x_sign_{1.0}, max_distance_{0.2}, force_limit_xy_{6.0}, max_speed_{0.4};
  double fx_{0}, fy_{0}, fz_{0};
  double initial_x_, initial_y_, initial_z_;

  // run flags
  bool started_{false}, initialized_{false}, published_init_{false};

  // completion signaling
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