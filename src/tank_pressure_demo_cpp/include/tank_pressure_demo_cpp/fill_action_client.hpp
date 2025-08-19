#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <functional>
#include <memory>
#include <string>

#include "hyfleet_interfaces/action/fill_to_target.hpp"

class FillActionClient : public rclcpp::Node {
public:
  using Fill           = hyfleet_interfaces::action::FillToTarget;
  using GoalHandleFill = rclcpp_action::ClientGoalHandle<Fill>;

  explicit FillActionClient(float target_bar);

  void send_goal();

private:
  void feedback_cb_(GoalHandleFill::SharedPtr,
                    const std::shared_ptr<const Fill::Feedback> fb);
  void result_cb_(const GoalHandleFill::WrappedResult & result);

  rclcpp_action::Client<Fill>::SharedPtr client_;
  Fill::Goal goal_;
};
