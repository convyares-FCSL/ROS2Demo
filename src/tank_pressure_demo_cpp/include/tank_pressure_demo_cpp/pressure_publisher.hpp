#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>   // needed in header
#include <random>
#include <string>

namespace tpd {

enum class BankId : uint8_t { BankA = 0, BankB = 1, BankC = 2 };

struct PressureConfig {
  float mean_bar = 700.0f;
  float jitter_bar = 20.0f;
  std::chrono::milliseconds period{500};
  std::string topic = "tank_pressure_bar";
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
};

struct ParameterKeys {
  static constexpr auto topic      = "topic";
  static constexpr auto mean_bar   = "mean_bar";
  static constexpr auto jitter_bar = "jitter_bar";
  static constexpr auto period_ms  = "period_ms";
  static constexpr auto bank_id    = "bank_id";
};

class TankPressurePublisher : public rclcpp::Node {
public:
  explicit TankPressurePublisher(const PressureConfig& cfg,
                                 BankId bank = BankId::BankA);

private:
  // lifecycle
  void declareStaticParams_();
  void loadParamsOnce_();
  void setupPublisherAndTimer_();
  void installParamCallback_();

  // helpers
  static BankId parseBankId_(const std::string& s);
  rcl_interfaces::msg::SetParametersResult
  onParamsSet_(const std::vector<rclcpp::Parameter>& params);

  // periodic
  void tick();

  // state
  PressureConfig cfg_;
  BankId bank_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<float> dist_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

} // namespace tpd
