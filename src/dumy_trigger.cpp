#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using Trigger = std_srvs::srv::Trigger;

class DumyTrigger : public rclcpp::Node
{
public:
  DumyTrigger() : Node("dumy_trigger")
  {
    // パブリッシャーの作成: /raking_trigger トピックに std_msgs::msg::Bool をパブリッシュ
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("/raking_trigger", 10);

    // サービスの作成: /dumy_trigger サービスを提供
    service_ = this->create_service<Trigger>(
      "/dumy_trigger",
      std::bind(&DumyTrigger::handle_trigger, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "DumyTrigger が起動しました。");
    RCLCPP_INFO(this->get_logger(), "サービス /dumy_trigger を待機中です。");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::Service<Trigger>::SharedPtr service_;

  // サービスのコールバック関数
  void handle_trigger(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    (void)request; // requestは使用しないが、引数として必要

    // 1. trueメッセージを作成
    auto bool_msg = std_msgs::msg::Bool();
    bool_msg.data = true;

    // 2. メッセージをパブリッシュ
    publisher_->publish(bool_msg);

    // 3. ログを出力
    RCLCPP_INFO(this->get_logger(), "サービスが呼ばれました: true を /raking_trigger にパブリッシュしました。");

    // 4. サービス応答を返す
    response->success = true;
    response->message = "Published 'true' to /raking_trigger topic.";
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DumyTrigger>());
  rclcpp::shutdown();
  return 0;
}