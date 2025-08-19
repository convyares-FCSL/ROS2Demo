#include "tank_pressure_demo_cpp/fill_action_server.hpp"
#include <algorithm>
using namespace std::chrono_literals;

FillActionServer::FillActionServer()
: rclcpp::Node("fill_action_server")
{
  this->declare_parameter<double>("start_bar", 96.0);
  this->declare_parameter<double>("ramp_bar_per_s", 1.0);
  this->declare_parameter<int>("tick_ms", 100);
  this->declare_parameter<double>("aprr_mpa_per_min", -1.0);

  actionServer_ = rclcpp_action::create_server<FillToTarget>(
    this,
    "/fill_to_target",
    std::bind(&FillActionServer::handleGoal,     this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&FillActionServer::handleCancel,   this, std::placeholders::_1),
    std::bind(&FillActionServer::handleAccepted, this, std::placeholders::_1)
  );

  pressurePub_ = this->create_publisher<std_msgs::msg::Float64>("/tank_pressure", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Action server ready on /fill_to_target");
}

rclcpp_action::GoalResponse
FillActionServer::handleGoal(const rclcpp_action::GoalUUID &,
                             std::shared_ptr<const FillToTarget::Goal> goal)
{
this->get_parameter("aprr_mpa_per_min", aprrMpaPerMin_);
this->get_parameter("start_bar",      startBar_);
this->get_parameter("ramp_bar_per_s", rampBarPerS_);
this->get_parameter("tick_ms",        tickMs_);

  if (goalActive_) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: another goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // If APRR is set, override ramp_bar_per_s
  if (aprrMpaPerMin_ > 0.0) {
    rampBarPerS_ = (aprrMpaPerMin_ * 10.0) / 60.0;  // MPa/min → bar/s
    RCLCPP_INFO(this->get_logger(),
      "APRR=%.3f MPa/min → ramp=%.3f bar/s", aprrMpaPerMin_, rampBarPerS_);
  }

  targetBar_ = goal->target_bar;
  RCLCPP_INFO(this->get_logger(), "Received goal request with target pressure %.1f bar", targetBar_);

  if (targetBar_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: target_bar must be > 0");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
FillActionServer::handleCancel(const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Cancel request received");
  stopRamp();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FillActionServer::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  activeGoal_ = goal_handle;
  startRamp();
}

void FillActionServer::startRamp()
{
  goalActive_ = true;
  currentBar_ = startBar_;
  startTime_ = this->now();   // mark start time]

  // Recreate timer with current period
  rampTimer_.reset();
  rampTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(tickMs_),
      std::bind(&FillActionServer::onRampTick, this));
}

void FillActionServer::stopRamp()
{
  if (rampTimer_) {
    rampTimer_->cancel();
    rampTimer_.reset();
  }
  goalActive_ = false;
  activeGoal_.reset();
}

void FillActionServer::onRampTick()
{
  if (!goalActive_ || !activeGoal_) return;

  auto gh = activeGoal_;
  if (gh->is_canceling()) {
    auto result = std::make_shared<FillToTarget::Result>();
    result->success = false;
    result->message = "Fill canceled";
    result->final_pressure_bar = currentBar_;
    gh->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal canceled at %.2f bar", currentBar_);
    stopRamp();
    return;
  }

  const double dt = static_cast<double>(tickMs_) / 1000.0;
  currentBar_ = std::min(currentBar_ + rampBarPerS_ * dt, targetBar_);
  double elapsed = (this->now() - startTime_).seconds();

  // publish current pressure
  std_msgs::msg::Float64 m;
  m.data = currentBar_;
  pressurePub_->publish(m);

  // Update action feedback
  FillToTarget::Feedback fb;
  fb.current_pressure_bar = currentBar_;
  fb.elapsed_s = elapsed;   // <-- set elapsed
  gh->publish_feedback(std::make_shared<FillToTarget::Feedback>(fb));

  if (currentBar_ >= targetBar_ - 1e-9) {
    auto result = std::make_shared<FillToTarget::Result>();
    result->success = true;
    result->message = "Fill completed successfully";
    result->final_pressure_bar = currentBar_;   // <- important
    gh->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Fill complete at %.2f bar", currentBar_);
    stopRamp();
  }
}

// ---------- main() at bottom, same file ----------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FillActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
