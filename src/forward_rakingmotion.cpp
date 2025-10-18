#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <executor_msgs/srv/plan.hpp>

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
  ServoForwardNode() : Node("servo_forward_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // ご希望通りデフォルトは base_link / tool0
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    tool_frame_ = this->declare_parameter<std::string>("tool_frame", "tool0");
    std::string direction = this->declare_parameter<std::string>("direction", "R");
    linear_x_sign_ = (direction == "R") ? 1.0 : -1.0;
    max_distance_   = this->declare_parameter<double>("max_distance", 0.2);
    force_limit_xy_ = this->declare_parameter<double>("force_limit_xy", 6.0);

    cg_srv_   = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cg_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);


    pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
    pub_dist_  = this->create_publisher<geometry_msgs::msg::PointStamped>("/distance_from_start", 10);
    sub_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/calibrated_force_data", 10, std::bind(&ServoForwardNode::force_cb, this, std::placeholders::_1));

    // /initial_pose をラッチ配信（back が後から起動しても届く）
    rclcpp::QoS latched(1); latched.transient_local().reliable();
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/initial_pose", latched);

 
    // rclcpp::ServiceOptions srv_opts;
    // srv_opts.callback_group = cg_srv_;
    srv_ = this->create_service<Plan>(
      "/forward/plan",
      std::bind(&ServoForwardNode::handle_plan, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      cg_srv_);
    // 制御はタイマーで駆動（サービスは開始合図だけ）
    timer_ = this->create_wall_timer(5ms, std::bind(&ServoForwardNode::on_timer, this), cg_timer_);

    RCLCPP_INFO(get_logger(), "[Forward] ready base=%s tool=%s dir=%s",
                base_frame_.c_str(), tool_frame_.c_str(), direction.c_str());
  }

private:
  // === service handler ===
  void handle_plan(const std::shared_ptr<Plan::Request> req, std::shared_ptr<Plan::Response> res)
  {
    RCLCPP_INFO(get_logger(), "[Forward] request: action=\"%s\" number=%d",
                req->action.c_str(), static_cast<int>(req->number));

    // 動作開始
    {
      std::lock_guard<std::mutex> lk(mtx_);
      started_ = true;
      initialized_ = false;
      published_init_ = false;
      waypoint_number_ = req->number;
      // 完了通知用の promise を新しく用意
      done_promise_.emplace();
      done_future_ = done_promise_->get_future();
    }

    // ★ ここで完了まで待ち、完了したら result を返す
    auto status = done_future_.wait_for(std::chrono::minutes(10)); // 必要ならタイムアウト調整
    std::string result = (status == std::future_status::ready) ? done_future_.get() : "timeout";
    res->result = result;
    RCLCPP_INFO(get_logger(), "[Forward] service done: result=\"%s\"", res->result.c_str());
  }

  // === timer loop ===
  void on_timer()
  {
    if (!started_) { publish_velocity(0.0); return; }

    if (!tf_buffer_.canTransform(base_frame_, tool_frame_, tf2::TimePointZero, 100ms)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF待ち: %s->%s", base_frame_.c_str(), tool_frame_.c_str());
      publish_velocity(0.0); return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try { tf = tf_buffer_.lookupTransform(base_frame_, tool_frame_, tf2::TimePointZero); }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF失敗: %s", ex.what());
      publish_velocity(0.0); return;
    }

    const double x = tf.transform.translation.x;
    const double y = tf.transform.translation.y;
    const double z = tf.transform.translation.z;

    if (!initialized_) {
      ix_ = x; iy_ = y; iz_ = z;
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "[Forward] init (%.3f, %.3f, %.3f)", x, y, z);
    }
    if (!published_init_) {
      geometry_msgs::msg::PointStamped init;
      init.header.stamp = now(); init.header.frame_id = base_frame_;
      init.point.x = ix_; init.point.y = iy_; init.point.z = iz_;
      initial_pose_pub_->publish(init);
      published_init_ = true;
      RCLCPP_INFO(get_logger(), "[Forward] /initial_pose latched");
    }

    const double dist = std::sqrt((x-ix_)*(x-ix_) + (y-iy_)*(y-iy_) + (z-iz_)*(z-iz_));
    publish_distance(dist);

    const double fxy = std::sqrt(fx_*fx_ + fy_*fy_);
    const double v   = std::max(0.0, 1.0 - std::min(fxy, force_limit_xy_) / force_limit_xy_);

    if (dist >= max_distance_ || fxy >= force_limit_xy_) {
      publish_velocity(0.0);

      std::lock_guard<std::mutex> lk(mtx_);
      started_ = false;
      if (done_promise_) { done_promise_->set_value("raking_end"); done_promise_.reset(); }
      return;
    }

    publish_velocity(linear_x_sign_ * v * max_speed_);
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
    p.point.x = d;
    pub_dist_->publish(p);
  }

  // pubs/subs/timer
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  pub_dist_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  initial_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_force_;
  rclcpp::Service<Plan>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr cg_srv_;
  rclcpp::CallbackGroup::SharedPtr cg_timer_;


  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // params / state
  std::string base_frame_, tool_frame_;
  double linear_x_sign_{1.0};
  double max_distance_{0.2};
  double force_limit_xy_{6.0};
  double max_speed_{0.5};

  double ix_{0}, iy_{0}, iz_{0};
  double fx_{0}, fy_{0}, fz_{0};

  // run flags
  bool started_{false}, initialized_{false}, published_init_{false};
  int  waypoint_number_{1};

  // completion signaling
  std::mutex mtx_;
  std::optional<std::promise<std::string>> done_promise_{};
  std::future<std::string> done_future_;

};

// ★ MultiThreadedExecutor で並行実行（サービス待機中もタイマー/購読が動く）
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoForwardNode>();
  rclcpp::executors::MultiThreadedExecutor exec;  // ★重要：マルチスレッド
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

