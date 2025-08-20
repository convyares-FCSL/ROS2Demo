#include "tank_pressure_demo_cpp/pressure_publisher_lc.hpp"
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::chrono::milliseconds;

namespace tpd {

PressurePublisherLC::PressurePublisherLC(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pressure_publisher_lc", options),
  diag_(this)
{
  // Declare parameters with defaults (values finalized in on_configure)
  this->declare_parameter("topic", topic_);
  this->declare_parameter("mean_bar", mean_bar_);
  this->declare_parameter("jitter_bar", jitter_bar_);
  this->declare_parameter("period_ms", period_ms_);
  this->declare_parameter("sp1_warn_bar", sp1_warn_bar_);
  this->declare_parameter("sp2_error_bar", sp2_error_bar_);

  // Diagnostics name/ID
  const std::string fq = std::string(this->get_namespace()) + "/" + this->get_name();
  diag_.setHardwareID(fq);
  diag_.add(fq + ": Pressure Status", this, &PressurePublisherLC::diagnosticsCb_);
}

CallbackReturn PressurePublisherLC::on_configure(const rclcpp_lifecycle::State &)
{
  // Read & validate parameters
  topic_        = this->get_parameter("topic").as_string();
  mean_bar_     = this->get_parameter("mean_bar").as_double();
  jitter_bar_   = this->get_parameter("jitter_bar").as_double();
  period_ms_    = this->get_parameter("period_ms").as_int();
  sp1_warn_bar_ = this->get_parameter("sp1_warn_bar").as_double();
  sp2_error_bar_= this->get_parameter("sp2_error_bar").as_double();

  if (period_ms_ <= 0 || jitter_bar_ < 0.0 || sp2_error_bar_ < sp1_warn_bar_) {
    RCLCPP_ERROR(get_logger(), "Invalid parameters");
    return CallbackReturn::FAILURE;
  }

  dist_ = std::uniform_real_distribution<float>(-static_cast<float>(jitter_bar_),
                                                 static_cast<float>(jitter_bar_));

  // Create lifecycle publisher
  pub_ = this->create_publisher<std_msgs::msg::Float32>(topic_, rclcpp::QoS{10});
  return CallbackReturn::SUCCESS;
}

CallbackReturn PressurePublisherLC::on_activate(const rclcpp_lifecycle::State &)
{
  pub_->on_activate();
  timer_ = this->create_wall_timer(milliseconds(period_ms_), [this]{ tick(); });
  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PressurePublisherLC::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (timer_) { timer_->cancel(); timer_.reset(); }
  if (pub_)   { pub_->on_deactivate(); }
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PressurePublisherLC::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PressurePublisherLC::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();
  RCLCPP_INFO(get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

void PressurePublisherLC::tick()
{
  const float p = static_cast<float>(mean_bar_) + dist_(gen_);
  std_msgs::msg::Float32 msg; msg.data = p;
  if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    pub_->publish(msg);
  }
  last_pressure_bar_ = p;
  diag_.force_update();
}

void PressurePublisherLC::diagnosticsCb_(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const double p = last_pressure_bar_;
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (p > sp2_error_bar_) {
    stat.summary(DiagnosticStatus::ERROR, "Pressure above SP2");
  } else if (p > sp1_warn_bar_) {
    stat.summary(DiagnosticStatus::WARN, "Pressure above SP1");
  } else {
    stat.summary(DiagnosticStatus::OK, "Pressure within limits");
  }
  stat.add("pressure_bar", p);
  stat.add("sp1_warn_bar", sp1_warn_bar_);
  stat.add("sp2_error_bar", sp2_error_bar_);
}

} // namespace tpd

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<tpd::PressurePublisherLC>();
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
