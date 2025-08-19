#pragma once

#include <chrono>
#include <memory>
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hyfleet_interfaces/action/fill_to_target.hpp"

class FillActionServer : public rclcpp::Node
{
public:
  using FillToTarget = hyfleet_interfaces::action::FillToTarget;
  using GoalHandle   = rclcpp_action::ServerGoalHandle<FillToTarget>;

  FillActionServer();

private:
  // Action
  rclcpp_action::Server<FillToTarget>::SharedPtr actionServer_;
  std::shared_ptr<GoalHandle> activeGoal_;

  // Ramp timer
  rclcpp::TimerBase::SharedPtr rampTimer_;
  rclcpp::Time startTime_;

  // Goal + state
  double targetBar_   = 0.0;
  double currentBar_  = 0.0;
  bool   goalActive_  = false;

  // Params
  double startBar_    = 96.0;
  double rampBarPerS_ = 1.0;
  int    tickMs_      = 100;
  double aprrMpaPerMin_ = -1.0;  // disabled when < 0

  // Action callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FillToTarget::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  // Ramp loop
  void startRamp();
  void stopRamp();
  void onRampTick();

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pressurePub_;
};
