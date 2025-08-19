#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <chrono>
#include <optional>

namespace schn {

struct Config {
  double low_bar_   = 680.0;
  double high_bar_  = 720.0;
  int    min_on_s_  = 10;   // seconds
  int    min_off_s_ = 10;   // seconds
  rclcpp::QoS qos   = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
};

struct ParameterKeys {
  static constexpr auto low_bar   = "low_bar";
  static constexpr auto high_bar  = "high_bar";
  static constexpr auto min_on_s  = "min_on_s";
  static constexpr auto min_off_s = "min_off_s";
  static constexpr auto topic     = "topic"; // reserved if you make the topic configurable later
};

// Hysteresis scheduler: start compressor below low_bar; stop above high_bar.
class SchedulerNode : public rclcpp::Node {
public:
  explicit SchedulerNode(const Config& cfg);

private:
  // periodic evaluation
  void tick_();

  // call the /start_compressor service
  void requestState_(bool want_running, const char* reason);

  // runtime parameter callback
  rcl_interfaces::msg::SetParametersResult
  onParamsSet_(const std::vector<rclcpp::Parameter>& params);

  // state/config
  Config cfg_;
  bool running_{false};

  using SteadyClock = std::chrono::steady_clock;
  using TimePoint   = std::chrono::time_point<SteadyClock>;

  std::optional<double>     last_pressure_;
  std::optional<TimePoint>  last_sample_time_;
  std::optional<TimePoint>  last_state_change_;

  // ROS handles
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

} // namespace schn
