#include "tank_pressure_demo_cpp/fill_action_server.hpp"

using namespace std::placeholders;

namespace tank_pressure_demo_cpp {

FillActionServer::FillActionServer() 
  : Node("fill_action_server") 
{
  sub_ = create_subscription<std_msgs::msg::Float32>(
    "pressure", 10,
    std::bind(&FillActionServer::pressure_callback_, this, _1));

  server_ = rclcpp_action::create_server<FillToTarget>(
    this,
    "fill_to_target",
    std::bind(&FillActionServer::handle_goal_, this, _1, _2),
    std::bind(&FillActionServer::handle_cancel_, this, _1),
    std::bind(&FillActionServer::execute_, this, _1));
}

void FillActionServer::pressure_callback_(const std_msgs::msg::Float32::SharedPtr msg) {
  current_pressure_ = msg->data;
}

rclcpp_action::GoalResponse FillActionServer::handle_goal_(
  const rclcpp_action::GoalUUID& uuid,
  std::shared_ptr<const FillToTarget::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal request with target pressure %.1f bar", goal->target_bar);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FillActionServer::handle_cancel_(
  const std::shared_ptr<GoalHandleFillToTarget> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FillActionServer::execute_(const std::shared_ptr<GoalHandleFillToTarget> goal_handle)
{
  auto feedback = std::make_shared<FillToTarget::Feedback>();
  auto result = std::make_shared<FillToTarget::Result>();
  
  // ... rest of execution logic ...
  
  result->success = true;
  result->message = "Fill completed successfully";
  result->final_pressure_bar = current_pressure_;
  
  goal_handle->succeed(result);
}

} // namespace tank_pressure_demo_cpp

// Add main function after namespace
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tank_pressure_demo_cpp::FillActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}