#include <rclcpp/rclcpp.hpp>
#include <executor_msgs/srv/plan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
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
    forward_srv_ = this->declare_parameter<std::string>("forward_service", "/forward/plan");
    back_srv_    = this->declare_parameter<std::string>("back_service", "/back/plan");

    forward_cli_ = this->create_client<Plan>(forward_srv_);
    back_cli_    = this->create_client<Plan>(back_srv_);

    // /initial_pose をラッチ購読（forward から受け取り）
    rclcpp::QoS latched(1); latched.transient_local().reliable();
    initial_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/initial_pose", latched, [this](geometry_msgs::msg::PointStamped::SharedPtr) {
        got_initial_.store(true);
        RCLCPP_INFO(this->get_logger(), "Got /initial_pose. Press any key to start BACK...");
      });

    // forward を1回送る（action="raking_start"）
    call_forward_();

    // キー監視スレッド
    start_key_thread_();

    // 状態確認タイマ
    timer_ = this->create_wall_timer(100ms, std::bind(&PlanClientRoundTrip::on_timer_, this));
  }

  ~PlanClientRoundTrip() override { stop_key_thread_(); }

private:
  void call_forward_()
  {
    if (!forward_cli_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "forward service not found: %s", forward_srv_.c_str());
      return;
    }
    auto req = std::make_shared<Plan::Request>();
    req->action = "raking_start";
    req->number = 1;

    auto fut = forward_cli_->async_send_request(req,
      [this](rclcpp::Client<Plan>::SharedFuture f){
        (void)f;
        RCLCPP_INFO(this->get_logger(), "[FORWARD] result received (should be 'raking_end').");
      });
    (void)fut;
    RCLCPP_INFO(get_logger(), "Called FORWARD (action='raking_start'). Waiting /initial_pose...");
  }

  void call_back_()
  {
    if (!back_cli_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "back service not found: %s", back_srv_.c_str());
      return;
    }
    auto req = std::make_shared<Plan::Request>();
    req->action = "raking_start"; // 区別したければ "raking_return_start"
    req->number = 1;

    auto fut = back_cli_->async_send_request(req,
      [this](rclcpp::Client<Plan>::SharedFuture f){
        (void)f;
        RCLCPP_INFO(this->get_logger(), "[BACK] result received (should be 'raking_end').");
      });
    (void)fut;
    RCLCPP_INFO(get_logger(), "Called BACK (action='raking_start').");
  }

  void on_timer_()
  {
    if (got_initial_.load() && !back_called_.load() && key_pressed_.load()) {
      back_called_.store(true);
      call_back_();
    }
  }

  // キー入力（非カノニカル・非ブロッキング）
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

  std::thread key_thread_; std::atomic<bool> key_stop_{false}; termios orig_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanClientRoundTrip>());
  rclcpp::shutdown();
  return 0;
}
