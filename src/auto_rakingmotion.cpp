#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <chrono>
using namespace std::chrono_literals;
using namespace std::chrono_literals;

class MoveAfterCollecting : public rclcpp::Node {
public:
  MoveAfterCollecting()
  : Node("move_after_detected_leaf"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    moving_(false), received_point_(false), current_step_(Step::X)
  {
    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/detected_leaf_edge_point",
        rclcpp::SensorDataQoS(),
        std::bind(&MoveAfterCollecting::point_callback, this, std::placeholders::_1));

    leaf_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/detected_leaf_point",
      rclcpp::SensorDataQoS(),
      std::bind(&MoveAfterCollecting::leaf_point_callback, this, std::placeholders::_1));
      
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10);

    force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/calibrated_force_data", 10,
      std::bind(&MoveAfterCollecting::force_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      50ms, std::bind(&MoveAfterCollecting::control_loop, this));

    wait_timer_ = this->create_wall_timer(
      1000ms, std::bind(&MoveAfterCollecting::check_sensor_ready, this)
    );

    RCLCPP_INFO(this->get_logger(), "/detected_leaf_pointを待機中...");
  }

private:
  enum class Step { X, Y, Z, R_X,R_R, WAIT,R_Y,DONE };
  Step current_step_;
  bool force_ready_ = false; // フォースセンサーデータ受信フラグ


  // ★WAIT処理のためのメンバ変数追加
  rclcpp::Time wait_start_time_;
  bool wait_started_ = false;

  void check_sensor_ready()
  {
    if (!force_ready_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for force sensor data on /calibrated_force_data ...");
    }
  }

  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (received_point_) return;
    // edge_point_ として記録
    edge_point_ = msg->point;
    has_edge_point_ = true;

  
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("base_link", "tool0", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "初期位置取得に失敗: %s", ex.what());
      return;
    }
  
    double initial_x = transform.transform.translation.x;
    double initial_y = transform.transform.translation.y;
    double initial_z = transform.transform.translation.z;
  
    const auto & pt = msg->point;
    if ((pt.x != 0.0 || pt.y != 0.0 || pt.z != 0.0)) {
      // 相対移動分を加算して目標位置を決定
      target_x_ = initial_x + pt.x;
      target_y_ = initial_y + pt.y;
      target_z_ = initial_z + pt.z;
  
      received_point_ = true;
      moving_ = true;
  
      RCLCPP_INFO(this->get_logger(),
        "目標座標受信（相対移動）: 初期(%.3f, %.3f, %.3f) + 相対(%.3f, %.3f, %.3f) → 目標(%.3f, %.3f, %.3f)",
        initial_x, initial_y, initial_z, pt.x, pt.y, pt.z,
        target_x_, target_y_, target_z_);
    }
  }

  void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    if (!force_ready_) {
      RCLCPP_INFO(this->get_logger(), "First force sensor data received.");
      force_ready_ = true;
    }
    fx = msg->wrench.force.x;
    fy = msg->wrench.force.y;
    fz = msg->wrench.force.z;
  }
  

  void control_loop() {
    if (!moving_ || current_step_ == Step::DONE) return;
    if (!force_ready_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,"Waiting for /calibrated_force_data ...");
      return; // センサ準備できるまでは制御をスキップ
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
    double dist=0;
    
    

      // 追加: 移動距離を常時出力
    double total_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    RCLCPP_INFO(this->get_logger(), "誤差: dx=%.3f, dy=%.3f, dz=%.3f, forece :fx=%.3f,fy=%.3f,fz=%.3f ", dx, dy, dz,fx,fy,fz);

    double threshold = 0.01;
    //double scale = 0.05;
    double  scale = velocity_calculation(abs_force); 

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->get_clock()->now();
    twist.header.frame_id = "base_link";

    switch (current_step_) {
      case Step::X:
        if (std::abs(dx) < threshold) {
          current_step_ = Step::Z;
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "X到達。次はYへ。");
          return;
        }
        twist.twist.linear.x = scale * (dx > 0 ? 1 : -1);
        break;
      
      case Step::Z:
      if (std::abs(dz) < threshold) {
        current_step_ = Step::Y;
        publish_stop();
        RCLCPP_INFO(this->get_logger(), "Y到達。移動完了。");
        return;
      }
      twist.twist.linear.z = scale * (dz > 0 ? 1 : -1);
      break;

      case Step::Y:
        if (std::abs(dy) < threshold) {
          publish_stop();
          RCLCPP_INFO(this->get_logger(), "Y到達。次はZへ。");
          current_step_ = Step::R_X;
         // rclcpp::shutdown();
          return;
        }
        twist.twist.linear.y = scale * (dy > 0 ? 1 : -1);
        break;
      
      case Step::R_X:
      // if(dist == 0) dist = current_x;
      // dist =  current_x-dist;
      if (std::abs(dx) >= 0.2 || abs_force >= 6.0) {
        current_step_ = Step::WAIT;
        wait_start_time_ = this->get_clock()->now();  // ★追加
        wait_started_ = true;      
        publish_stop();
        RCLCPP_INFO(this->get_logger(), "raking motion");
        return;
      }
      twist.twist.linear.x = scale * get_edge_direction();
      break;

     case Step::WAIT:  // ★追加：5秒待機処理

      if (wait_started_) {
        rclcpp::Duration elapsed = this->get_clock()->now() - wait_start_time_;
        if (elapsed.seconds() >= 5.0) {
          RCLCPP_INFO(this->get_logger(), "5秒待機完了 → DONE");
          current_step_ = Step::R_R;
          //rclcpp::shutdown();
        } else {
          publish_stop();  // ★停止維持
        }
      }
      break;

      case Step::R_R:
      // if(dist == 0) dist = current_x;
      // dist =  current_x-dist;
      if (std::abs(dx) <= threshold) {
        current_step_ = Step::R_Y;
        target_y_= (current_y-0.1);
        publish_stop();
        RCLCPP_INFO(this->get_logger(), "raking motion return");
        //rclcpp::shutdown();
        return;
      }
      twist.twist.linear.x = scale * get_edge_direction()*-1;
      break;

      case Step::R_Y:
      
      if (std::abs(dy) < threshold) {
        publish_stop();
        RCLCPP_INFO(this->get_logger(), "Y到達。次はZへ。");
        current_step_ =Step::DONE;
       rclcpp::shutdown();
        return;
      }
      twist.twist.linear.y = scale * (dy > 0 ? 1 : -1);
      break;
    
      



      case Step::DONE:
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
    twist_pub_->publish(twist);
  }

  bool exceeded_force()
  {
    //return std::abs(fx) > 6.0 || std::abs(fy) > 6.0 || std::abs(fz) > 6.0;
    return (std::abs(fx) + std::abs(fy)) > 6.0 ;
  } 

  double velocity_calculation(double force)
  {
    //RCLCPP_INFO(this->get_logger(), "forece:%.3f",force);
    if(force>=6.0)force=6.0;
    // double velocity=(150*(1-((1/6)*force)))/1000;
    double velocity=1.0*(1.0-((1.0/6.0))*force);
    if(velocity >= 1.0) velocity=1.0;
    return velocity;
  }

  void leaf_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    leaf_point_ = msg->point;
    has_leaf_point_ = true;
  }
  

  int get_edge_direction() {
    if (!has_edge_point_ || !has_leaf_point_) {
      RCLCPP_WARN(this->get_logger(), "ポイント未取得: edge=%d, leaf=%d", has_edge_point_, has_leaf_point_);
      return 0;  // エラーや未取得時の無効値
    }
  
    double edge_x = edge_point_.x;
    double leaf_x = leaf_point_.x;
  
    if (edge_x < leaf_x) return +1;
    else if (edge_x > leaf_x) return -1;
    else return 0;
  }
  

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr leaf_point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::Point edge_point_;
  geometry_msgs::msg::Point leaf_point_;

  double target_x_, target_y_, target_z_;
  bool moving_;
  bool received_point_;
  double linear_x_;
  double fx = 0, fy = 0, fz = 0;
  double initial_x_, initial_y_, initial_z_;
  bool initialized_;
  bool has_edge_point_ = false;
  bool has_leaf_point_ = false;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveAfterCollecting>());
  rclcpp::shutdown();
  return 0;
}
