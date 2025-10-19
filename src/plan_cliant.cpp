#include <rclcpp/rclcpp.hpp>
#include <executor_msgs/srv/plan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

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

    // latched /initial_pose
    rclcpp::QoS latched(1); latched.transient_local().reliable();
    initial_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/initial_pose", latched, [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        // builtin_interfaces::msg::Time -> rclcpp::Time に変換して比較
        const rclcpp::Time t(msg->header.stamp);
        // 既に初期化済みかつ古い/同一スタンプは無視
        if (last_stamp_.nanoseconds() != 0 && t <= last_stamp_) {
          return;
        }
        last_stamp_ = t;
        got_initial_.store(true);
        RCLCPP_INFO(this->get_logger(), "Got /initial_pose. Press any key to start BACK...");
      });

    call_forward_();
    start_key_thread_();
    timer_ = this->create_wall_timer(100ms, std::bind(&PlanClientRoundTrip::on_timer_, this));
  }
  ~PlanClientRoundTrip() override { stop_key_thread_(); }

private:
  void call_forward_()
  {
    if (!forward_cli_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "forward service not found: %s", forward_srv_.c_str()); return;
    }
    auto req = std::make_shared<Plan::Request>();
    req->action = "raking_start"; req->number = 1;
    forward_cli_->async_send_request(req,
      [this](rclcpp::Client<Plan>::SharedFuture f) {
        auto res = f.get();
        RCLCPP_INFO(this->get_logger(), "[FORWARD] result: %s", res->result.c_str());
      });
    RCLCPP_INFO(get_logger(), "Called FORWARD (action='raking_start'). Waiting /initial_pose...");
  }

  void call_back_()
  {
    if (!back_cli_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "back service not found: %s", back_srv_.c_str()); return;
    }
    auto req = std::make_shared<Plan::Request>();
    req->action = "raking_start"; req->number = 1;
    back_cli_->async_send_request(req,
      [this](rclcpp::Client<Plan>::SharedFuture f) {
        auto res = f.get();
        if (res->result == "busy")
          RCLCPP_WARN(this->get_logger(), "[BACK] busy (ignored)");
        else
          RCLCPP_INFO(this->get_logger(), "[BACK] result: %s", res->result.c_str());
      });
    RCLCPP_INFO(get_logger(), "Called BACK (action='raking_start').");
  }

  void on_timer_()
  {
    if (got_initial_.load() && !back_called_.load() && key_pressed_.load()) {
      back_called_.store(true);
      call_back_();
    }
  }

  // non-blocking key reader
  void start_key_thread_()
  {
    tcgetattr(STDIN_FILENO, &orig_);
    termios raw = orig_; raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    key_stop_.store(false);
    key_thread_ = std::thread([this](){
      char c;
      while (rclcpp::ok() && !key_stop_.load()) {
        if (read(STDIN_FILENO, &c, 1) > 0) { key_pressed_.store(true); break; }
        std::this_thread::sleep_for(10ms);
      }
    });
  }
  void stop_key_thread_()
  {
    key_stop_.store(true);
    if (key_thread_.joinable()) key_thread_.join();
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
  }

  // members
  rclcpp::Client<Plan>::SharedPtr forward_cli_, back_cli_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr initial_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string forward_srv_, back_srv_;
  std::atomic<bool> got_initial_{false}, key_pressed_{false}, back_called_{false};
  rclcpp::Time last_stamp_;

  std::thread key_thread_; std::atomic<bool> key_stop_{false}; termios orig_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanClientRoundTrip>());
  rclcpp::shutdown();
  return 0;
}
