#ifndef FILL_ACTION_SERVER_HPP_
#define FILL_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <hyfleet_interfaces/action/fill_to_target.hpp>

namespace tank_pressure_demo_cpp {

using FillToTarget = hyfleet_interfaces::action::FillToTarget;
using GoalHandleFillToTarget = rclcpp_action::ServerGoalHandle<FillToTarget>;

class FillActionServer : public rclcpp::Node {
public:
  FillActionServer();

private:
  // Member variables
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp_action::Server<FillToTarget>::SharedPtr server_;
  float current_pressure_{0.0f};

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal_(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FillToTarget::Goal> goal);
    
  rclcpp_action::CancelResponse handle_cancel_(
    const std::shared_ptr<GoalHandleFillToTarget> goal_handle);
    
  void execute_(const std::shared_ptr<GoalHandleFillToTarget> goal_handle);

  // Subscription callback
  void pressure_callback_(const std_msgs::msg::Float32::SharedPtr msg);
};

} // namespace tank_pressure_demo_cpp

#endif  // FILL_ACTION_SERVER_HPP_