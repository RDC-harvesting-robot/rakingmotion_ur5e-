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

class ServoBackNode : public rclcpp::Node
{
public:
ServoBackNode() : Node("servo_back_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  base_frame_     = this->declare_parameter<std::string>("base_frame", "left_arm_base_link_inertia");
  tool_frame_     = this->declare_parameter<std::string>("tool_frame", "left_armtool0");
  back_tolerance_ = this->declare_parameter<double>("back_tolerance", 0.01);
  force_limit_xy_ = this->declare_parameter<double>("force_limit_xy", 6.0);


  cg_srv_   = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cg_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/left_arm/servo_node/delta_twist_cmds", 10);
  sub_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/left/calibrated_force_data", 10, std::bind(&ServoBackNode::force_cb, this, std::placeholders::_1));

  rclcpp::QoS latched(1); latched.transient_local().reliable();
  sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/initial_pose", latched, std::bind(&ServoBackNode::initial_pose_cb, this, std::placeholders::_1));


  srv_ = this->create_service<Plan>(
      "/back/plan",
      std::bind(&ServoBackNode::handle_plan, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      cg_srv_);


  timer_ = this->create_wall_timer(1ms, std::bind(&ServoBackNode::on_timer, this), cg_timer_);

  RCLCPP_INFO(get_logger(), "[Back] ready base=%s tool=%s", base_frame_.c_str(), tool_frame_.c_str());
}


private:
  void handle_plan(const std::shared_ptr<Plan::Request> req, std::shared_ptr<Plan::Response> res)
  {
    RCLCPP_INFO(get_logger(), "[Back] request: action=\"%s\" number=%d",
                req->action.c_str(), static_cast<int>(req->number));

    {
      std::lock_guard<std::mutex> lk(mtx_);
      started_ = true;
      waypoint_number_ = req->number;
      if (!have_goal_) RCLCPP_WARN(get_logger(), "まだ /initial_pose を受信していません。受信待ちで停止中。");
      done_promise_.emplace();
      done_future_ = done_promise_->get_future();
    }

    auto status = done_future_.wait_for(std::chrono::minutes(10));
    std::string result = (status == std::future_status::ready) ? done_future_.get() : "timeout";
    res->result = result;
    RCLCPP_INFO(get_logger(), "[Back] service done: result=\"%s\"", res->result.c_str());
  }

  void initial_pose_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != base_frame_) {
      try {
        auto tf = tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero);
        geometry_msgs::msg::PointStamped p_out; tf2::doTransform(*msg, p_out, tf);
        goal_x_ = p_out.point.x; goal_y_ = p_out.point.y; goal_z_ = p_out.point.z;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "initial_pose 変換失敗: %s", ex.what());
        return;
      }
    } else {
      goal_x_ = msg->point.x; goal_y_ = msg->point.y; goal_z_ = msg->point.z;
    }
    have_goal_ = true;
    RCLCPP_INFO(get_logger(), "[Back] goal set (%.3f, %.3f, %.3f)", goal_x_, goal_y_, goal_z_);
  }

  void on_timer()
  {
    static int state_=0;
    if (!started_) {  return; }
    if (!have_goal_) {  return; }
    if(state_== 0)state_ = 1;

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
    const double dx = x - goal_x_, dy = y - goal_y_, dz = z - goal_z_;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    const double fxy = std::sqrt(fx_*fx_ + fy_*fy_);
    const double v   = std::max(0.0, 1.0 - std::min(fxy, force_limit_xy_) / force_limit_xy_);
    double velocity=0;
    // double abs_force=std::sqrt((fx_*fx_)+(fy_*fy_));
    // velocity = velocity_calculation(abs_force); 
    velocity = velocity_calculation(abs_force); 
    double dist_2 = std::sqrt(
      std::pow(x - initial_x_, 2) +
      std::pow(y - initial_y_, 2) +
      std::pow(z - initial_z_, 2));

    switch (state_) {
      case 1:
        if (dist <= back_tolerance_) {
          publish_velocity(0.0);
          RCLCPP_INFO(get_logger(), "[Back] reached goal (%.3f m)", dist);
          initial_x_ = x;
          initial_y_ = y;
          initial_z_ = z;
          state_=2;
          break;
        } else {
          const double sign_x = (dx > 0.0) ? -1.0 : 1.0;
          RCLCPP_INFO(get_logger(), "[Back] reached goal (%.3f m)", dist);
          publish_velocity(velocity);
        }
        break;
      case 2:
        if (dist_2 >= 0.3 ) {
          RCLCPP_INFO(this->get_logger(), "前進");
          publish_stop();
          std::lock_guard<std::mutex> lk(mtx_);
          started_ = false;
          if (done_promise_) { done_promise_->set_value("return_raking_end"); done_promise_.reset(); }
          rclcpp::shutdown();
          return;
        } else {
          publish_velocity_y(1.0);
          RCLCPP_INFO(get_logger(), "[Back] reached goal_________ (%.3f m)", dist_2);
        }
        break;
    }

  
  }

  void force_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  { 
    fx_ = msg->wrench.force.x;
    fy_ = msg->wrench.force.y;
    fz_ = msg->wrench.force.z; 
    
   abs_force=std::sqrt((fx_*fx_)+(fy_*fy_));
   //RCLCPP_INFO(this->get_logger(), "forece_X_Y:%.3f",abs_force);
  }

  void publish_velocity(double vx)
  {
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now(); m.header.frame_id = base_frame_;
    m.twist.linear.x = vx;
    pub_twist_->publish(m);
  }

  void publish_velocity_y(double vy)
  {
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now(); m.header.frame_id = base_frame_;
    m.twist.linear.y = vy;
    pub_twist_->publish(m);
  }

  double velocity_calculation(double force)
  {
    //RCLCPP_INFO(this->get_logger(), "forece:%.3f",force);
    if(force>=force_limit_xy_)force=force_limit_xy_;
    // double velocity=(150*(1-((1/6)*force)))/1000;
    double velocity=1.0*(1.0-((1.0/force_limit_xy_))*force);
    if(velocity >= 1.0) velocity=1.0;
    return -1*velocity;
  }
  void publish_stop()
  { 
    publish_velocity(0.0);
  }

  bool exceeded_force(double force)
  {
    static double force_buff=0;
    //return std::abs(fx) > 6.0 || std::abs(fy) > 6.0 || std::abs(fz) > 6.0;
    if(force >= force_limit_xy_ && force_buff >= force_limit_xy_ && force_buff != force){
      return 1;
  
    }else {
      force_buff =  force;
      return 0;
    }
   
  } 


  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_force_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr  sub_initial_pose_;
  rclcpp::Service<Plan>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr cg_srv_;
  rclcpp::CallbackGroup::SharedPtr cg_timer_;



  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;


  std::string base_frame_, tool_frame_;
  double back_tolerance_{0.01}, force_limit_xy_{6.0}, max_speed_{0.5};
  double fx_{0}, fy_{0}, fz_{0};
  bool started_{false}, have_goal_{false};
  int  waypoint_number_{1};
  double goal_x_{0}, goal_y_{0}, goal_z_{0};
  double initial_x_, initial_y_, initial_z_;
  double abs_force;

  std::mutex mtx_;
  std::optional<std::promise<std::string>> done_promise_{};
  std::future<std::string> done_future_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoBackNode>();
  rclcpp::executors::MultiThreadedExecutor exec;  
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

