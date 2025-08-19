#include "tank_pressure_demo_cpp/pressure_publisher.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <optional>

using std::chrono::milliseconds;
namespace tpd {

TankPressurePublisher::TankPressurePublisher(const PressureConfig& cfg, BankId bank)
: rclcpp::Node("tank_pressure_pub_cpp"),
  cfg_(cfg),
  bank_(bank),
  gen_(rd_()),
  dist_(-cfg.jitter_bar, cfg.jitter_bar)
{
  declareStaticParams_();
  loadParamsOnce_();
  setupPublisherAndTimer_();
  installParamCallback_();
}

void TankPressurePublisher::declareStaticParams_() {
  rcl_interfaces::msg::ParameterDescriptor d_mean;
  d_mean.name = ParameterKeys::mean_bar;

  rcl_interfaces::msg::ParameterDescriptor d_jitter;
  d_jitter.name = ParameterKeys::jitter_bar;
  d_jitter.floating_point_range = { rcl_interfaces::msg::FloatingPointRange{} };
  d_jitter.floating_point_range[0].from_value = 0.0;
  d_jitter.floating_point_range[0].to_value   = 1e6;
  d_jitter.floating_point_range[0].step       = 0.0;

  rcl_interfaces::msg::ParameterDescriptor d_period;
  d_period.name = ParameterKeys::period_ms;
  d_period.integer_range = { rcl_interfaces::msg::IntegerRange{} };
  d_period.integer_range[0].from_value = 1;
  d_period.integer_range[0].to_value   = 3600000;
  d_period.integer_range[0].step       = 1;

  rcl_interfaces::msg::ParameterDescriptor d_topic;
  d_topic.name = ParameterKeys::topic;

  rcl_interfaces::msg::ParameterDescriptor d_bank;
  d_bank.name = ParameterKeys::bank_id;

  this->declare_parameter<std::string>(ParameterKeys::topic, cfg_.topic, d_topic);
  this->declare_parameter<double>(ParameterKeys::mean_bar, cfg_.mean_bar, d_mean);
  this->declare_parameter<double>(ParameterKeys::jitter_bar, cfg_.jitter_bar, d_jitter);
  this->declare_parameter<int>(ParameterKeys::period_ms,
                               static_cast<int>(cfg_.period.count()), d_period);
  this->declare_parameter<std::string>(ParameterKeys::bank_id, "BankA", d_bank);
}

void TankPressurePublisher::loadParamsOnce_() {
  cfg_.topic      = this->get_parameter(ParameterKeys::topic).as_string();
  cfg_.mean_bar   = static_cast<float>(this->get_parameter(ParameterKeys::mean_bar).as_double());
  cfg_.jitter_bar = static_cast<float>(this->get_parameter(ParameterKeys::jitter_bar).as_double());
  cfg_.period     = milliseconds(this->get_parameter(ParameterKeys::period_ms).as_int());

  const auto bank_str = this->get_parameter(ParameterKeys::bank_id).as_string();
  try {
    bank_ = parseBankId_(bank_str);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
      "Invalid bank_id '%s' (%s). Falling back to BankA.",
      bank_str.c_str(), e.what());
    bank_ = BankId::BankA;
  }

  dist_ = std::uniform_real_distribution<float>(-cfg_.jitter_bar, cfg_.jitter_bar);
}

void TankPressurePublisher::setupPublisherAndTimer_() {
  pub_ = this->create_publisher<std_msgs::msg::Float32>(cfg_.topic, cfg_.qos);
  timer_ = this->create_wall_timer(cfg_.period,
           std::bind(&TankPressurePublisher::tick, this));
}

void TankPressurePublisher::installParamCallback_() {
  param_cb_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      return this->onParamsSet_(params);
    });
}

rcl_interfaces::msg::SetParametersResult
TankPressurePublisher::onParamsSet_(const std::vector<rclcpp::Parameter>& params) {
  auto new_cfg = cfg_;
  std::optional<std::string> new_bank;

  for (const auto& p : params) {
    if (p.get_name() == ParameterKeys::mean_bar &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      new_cfg.mean_bar = static_cast<float>(p.as_double());

    } else if (p.get_name() == ParameterKeys::jitter_bar &&
               p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (p.as_double() < 0.0) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false;
        res.reason = "jitter_bar must be >= 0";
        return res;
      }
      new_cfg.jitter_bar = static_cast<float>(p.as_double());

    } else if (p.get_name() == ParameterKeys::period_ms &&
               p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (p.as_int() <= 0) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false;
        res.reason = "period_ms must be > 0";
        return res;
      }
      new_cfg.period = std::chrono::milliseconds(p.as_int());

    } else if (p.get_name() == ParameterKeys::bank_id &&
               p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      new_bank = p.as_string();
      if (*new_bank != "BankA" && *new_bank != "BankB" && *new_bank != "BankC") {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false;
        res.reason = "bank_id must be BankA|BankB|BankC";
        return res;
      }
    }
  }

  // Commit + reconfigure
  cfg_ = new_cfg;
  dist_ = std::uniform_real_distribution<float>(-cfg_.jitter_bar, cfg_.jitter_bar);

  if (timer_) {
    timer_.reset();
  }
  timer_ = this->create_wall_timer(cfg_.period, std::bind(&TankPressurePublisher::tick, this));

  if (new_bank) {
    bank_ = parseBankId_(*new_bank);
  }

  rcl_interfaces::msg::SetParametersResult ok;
  ok.successful = true;
  ok.reason = "";
  return ok;
}

BankId TankPressurePublisher::parseBankId_(const std::string& s) {
  if (s == "BankA") return BankId::BankA;
  if (s == "BankB") return BankId::BankB;
  if (s == "BankC") return BankId::BankC;
  throw std::invalid_argument("bank_id must be BankA|BankB|BankC");
}

void TankPressurePublisher::tick() {
  const float p = cfg_.mean_bar + dist_(gen_);
  std_msgs::msg::Float32 msg;
  msg.data = p;
  pub_->publish(msg);
}

} // namespace tpd

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  tpd::PressureConfig cfg;
  auto node = std::make_shared<tpd::TankPressurePublisher>(cfg, tpd::BankId::BankA);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
