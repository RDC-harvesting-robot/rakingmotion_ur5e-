#include <rclcpp/rclcpp.hpp>
#include <executor_msgs/srv/plan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

using Plan = executor_msgs::srv::Plan;
using namespace std::chrono_literals;

class PlanClientRoundTrip : public rclcpp::Node
{
public:
  PlanClientRoundTrip() : Node("plan_client_roundtrip")
  {
    last_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    forward_srv_ = this->declare_parameter<std::string>("forward_service", "/forward/plan");
    back_srv_    = this->declare_parameter<std::string>("back_service", "/back/plan");

    forward_cli_ = this->create_client<Plan>(forward_srv_);
    back_cli_    = this->create_client<Plan>(back_srv_);

    // /initial_pose はラッチ購読
    rclcpp::QoS latched(1); latched.transient_local().reliable();
    initial_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/initial_pose", latched,
      std::bind(&PlanClientRoundTrip::initial_pose_cb_, this, std::placeholders::_1));
  }

  void run_once()
  {
    // 1) forward を同期呼び出し
    if (!forward_cli_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "forward service not found: %s", forward_srv_.c_str());
      return;
    }
    auto f_req = std::make_shared<Plan::Request>();
    f_req->action = "raking_start";
    f_req->number = 1;

    RCLCPP_INFO(get_logger(), "Calling FORWARD… (action='%s')", f_req->action.c_str());
    auto f_fut = forward_cli_->async_send_request(f_req);
    if (rclcpp::spin_until_future_complete(shared_from_this(), f_fut, 10min)
          != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "FORWARD call timed out / failed");
      return;
    }
    RCLCPP_INFO(get_logger(), "FORWARD result: %s", f_fut.get()->result.c_str());


    RCLCPP_INFO(get_logger(), "Waiting for /initial_pose (latched) …");
    while (rclcpp::ok() && !got_initial_) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(20ms);
    }
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(get_logger(), "Got /initial_pose. Press any key to start BACK…");

   
    enable_raw_mode_();
    while (rclcpp::ok()) {
      
      rclcpp::spin_some(shared_from_this());

      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(STDIN_FILENO, &rfds);
      timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 100000;

      int ret = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
      if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
        char c;
        (void)::read(STDIN_FILENO, &c, 1);
        break;
      }
    }
    disable_raw_mode_();
    if (!rclcpp::ok()) return;

  
    if (!back_cli_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "back service not found: %s", back_srv_.c_str());
      return;
    }
    auto b_req = std::make_shared<Plan::Request>();
    b_req->action = "raking_start";
    b_req->number = 1;

    RCLCPP_INFO(get_logger(), "Calling BACK… (action='%s')", b_req->action.c_str());
    auto b_fut = back_cli_->async_send_request(b_req);
    if (rclcpp::spin_until_future_complete(shared_from_this(), b_fut, 10min)
          != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "BACK call timed out / failed");
      return;
    }
    RCLCPP_INFO(get_logger(), "BACK result: %s", b_fut.get()->result.c_str());
  }

private:
  void initial_pose_cb_(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
  
    rclcpp::Time t(msg->header.stamp, this->get_clock()->get_clock_type());
    if (last_stamp_.nanoseconds() != 0 && t <= last_stamp_) return;
    last_stamp_ = t;
    got_initial_ = true;
  }

  
  void enable_raw_mode_()
  {
    tcgetattr(STDIN_FILENO, &orig_termios_);
    termios raw = orig_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  void disable_raw_mode_()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
  }

  // members
  rclcpp::Client<Plan>::SharedPtr forward_cli_, back_cli_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr initial_sub_;
  std::string forward_srv_, back_srv_;

  bool got_initial_{false};
  rclcpp::Time last_stamp_;
  termios orig_termios_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanClientRoundTrip>();
  node->run_once(); 
  rclcpp::shutdown();
  return 0;
}
