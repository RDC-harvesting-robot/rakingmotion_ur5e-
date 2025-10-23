#include <rclcpp/rclcpp.hpp>
#include <executor_msgs/srv/plan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <chrono>

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

    rclcpp::QoS latched(1); latched.transient_local().reliable();
    initial_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/initial_pose", latched,
      std::bind(&PlanClientRoundTrip::initial_pose_cb_, this, std::placeholders::_1));

    hr_client_ = this->create_client<executor_msgs::srv::Plan>("/execute_plan_demo");
    start_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PlanClientRoundTrip::start_timer_demo_forward_rakingmotion, this));
  }


  void run_once()
  {
    forward_rakingmotion();
    return_rakingmotion();
  }

private:
  void initial_pose_cb_(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    rclcpp::Time t(msg->header.stamp, this->get_clock()->get_clock_type());
    if (last_stamp_.nanoseconds() != 0 && t <= last_stamp_) return; 
    last_stamp_ = t;
    got_initial_ = true;
  }


  void forward_rakingmotion(){
    if (!forward_cli_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "forward service not found: %s", forward_srv_.c_str());
      return;
    }
    auto f_req = std::make_shared<Plan::Request>();
    f_req->action = "raking_start";
    f_req->number = 1;

    RCLCPP_INFO(get_logger(), "Calling FORWARD (async) …");
    forward_cli_->async_send_request(
    f_req,
    std::bind(&PlanClientRoundTrip::response_callback, this, std::placeholders::_1));
  }

  void return_rakingmotion(){

    // ----------------------returnを呼び出し-------------------------------
    if (!back_cli_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "back service not found: %s", back_srv_.c_str());
      return;
    }
    auto b_req = std::make_shared<Plan::Request>();
    b_req->action = "raking_start";
    b_req->number = 1;

    RCLCPP_INFO(get_logger(), "Calling BACK (async) …");
    back_cli_->async_send_request(
    b_req,
    std::bind(&PlanClientRoundTrip::response_callback, this, std::placeholders::_1));
  }
  
//---demo_hr---
  void demo_hr()
    {  
      // サーバーが起動するまで待つ
        if (!hr_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "サービスが見つかりません(demo_hr)");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "demo_hr");
        auto request = std::make_shared<executor_msgs::srv::Plan::Request>();
        request->action = "demo_hr";
        auto result_future = hr_client_->async_send_request(request,
                std::bind(&PlanClientRoundTrip::response_callback, this, std::placeholders::_1));
    }

  void return_demo_hr()
  {  
    // サーバーが起動するまで待つ
      if (!hr_client_->wait_for_service(std::chrono::seconds(5))) {
          RCLCPP_ERROR(this->get_logger(), "サービスが見つかりません(return_demo_hr)");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "return_demo_hr");
      auto request = std::make_shared<executor_msgs::srv::Plan::Request>();
      request->action = "return_demo_hr";
      auto result_future = hr_client_->async_send_request(request,
              std::bind(&PlanClientRoundTrip::response_callback, this, std::placeholders::_1));
  }

  void wait_key()
  {
    RCLCPP_INFO(this->get_logger(), "wait p key");
    int key;
    while(rclcpp::ok())
    {
      key = std::getchar();
      if (key == 'p'){
        return;
      }
    }
  }

//行動計画
  void start_timer_demo_forward_rakingmotion()
  {
    this->start_timer_->cancel();
    forward_rakingmotion();
    // // サーバーが起動するまで待つ
    // if (!hr_client_->wait_for_service(std::chrono::seconds(5))) {
    //   RCLCPP_ERROR(this->get_logger(), "サービスが見つかりません(demo_hr)");
    //   return;
    // }
    
    // RCLCPP_INFO(this->get_logger(), "Waiting for target position...");
    // auto request = std::make_shared<executor_msgs::srv::Plan::Request>();
    // request->action = "demo_hr";
    // auto result_future = hr_client_->async_send_request(request,
    //   std::bind(&PlanClient::response_callback, this, std::placeholders::_1));
    // RCLCPP_INFO(this->get_logger(), "start_demo");
  }
  void response_callback(rclcpp::Client<executor_msgs::srv::Plan>::SharedFuture result_future)
  {
    try
    {
      auto response = result_future.get();
      RCLCPP_INFO(this->get_logger(), "サービスからの応答を受信");
      if (response->result == "raking_end"){
        RCLCPP_INFO(this->get_logger(), "raking_end -> demo_hr");
        this->demo_hr();
      }
      if (response->result == "demo_hr_SUCCESS"){
        RCLCPP_INFO(this->get_logger(), "demo_hr_SUCCESS -> return by key");
        this->wait_key();
        this->return_demo_hr();
      }
      else if(response->result == "return_demo_hr_SUCCESS"){
        RCLCPP_INFO(this->get_logger(), "return_demo_hr_SUCCESS -> return_ranking");
        this->return_rakingmotion();
      }
      else if(response->result == "return_raking_end"){
        RCLCPP_INFO(this->get_logger(), "return_raking_end -> finish");
        this->wait_key();
      }
      else if(response->result == "demo_hr_FAILURE"){ // arudake
        RCLCPP_INFO(this->get_logger(), "demo_hr_FAILURE");
        this->wait_key();
      }
      else{
        RCLCPP_WARN(this->get_logger(), "エラー:%s", response->result.c_str());
        return;
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "サービス呼び出し中にエラー: %s", e.what());
    }
  }


  rclcpp::Client<Plan>::SharedPtr forward_cli_, back_cli_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr initial_sub_;
  std::string forward_srv_, back_srv_;
  bool got_initial_{false};
  rclcpp::Time last_stamp_;

  rclcpp::Client<executor_msgs::srv::Plan>::SharedPtr hr_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanClientRoundTrip>();
  // node->run_once();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
