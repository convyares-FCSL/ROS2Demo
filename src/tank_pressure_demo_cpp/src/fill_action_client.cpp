#include "tank_pressure_demo_cpp/fill_action_client.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

FillActionClient::FillActionClient(float target_bar)
: rclcpp::Node("fill_action_client")
{
  client_ = rclcpp_action::create_client<Fill>(this, "fill_to_target");
  goal_.target_bar = target_bar;
}

void FillActionClient::send_goal() {
  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "Action server not available");
    rclcpp::shutdown();
    return;
  }

  rclcpp_action::Client<Fill>::SendGoalOptions opts;
  opts.feedback_callback = std::bind(&FillActionClient::feedback_cb_, this,
                                     std::placeholders::_1, std::placeholders::_2);
  opts.result_callback   = std::bind(&FillActionClient::result_cb_, this,
                                     std::placeholders::_1);

  client_->async_send_goal(goal_, opts);
}

void FillActionClient::feedback_cb_(GoalHandleFill::SharedPtr,
                                    const std::shared_ptr<const Fill::Feedback> fb)
{
  RCLCPP_INFO(get_logger(), "Feedback: p=%.1f bar, t=%.1fs",
              fb->current_pressure_bar, fb->elapsed_s);
}

void FillActionClient::result_cb_(const GoalHandleFill::WrappedResult & result)
{
  const auto & res = result.result;
  RCLCPP_INFO(get_logger(), "Result (%d): %s (final=%.1f bar)",
              static_cast<int>(result.code), res->message.c_str(), res->final_pressure_bar);
  // Optional: shutdown after first goal completes
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  float target = 705.0f;
  if (argc > 1) {
    try { target = std::stof(argv[1]); } catch (...) {}
  }
  auto node = std::make_shared<FillActionClient>(target);
  node->send_goal();
  rclcpp::spin(node);
  return 0;
}
