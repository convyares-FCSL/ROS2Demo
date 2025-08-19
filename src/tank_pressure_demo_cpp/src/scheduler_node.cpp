#include "tank_pressure_demo_cpp/scheduler_node.hpp"
#include <chrono>

using namespace std::chrono_literals;  // enables 2s, 0s, etc.

namespace schn {

SchedulerNode::SchedulerNode(const Config& cfg)
: rclcpp::Node("scheduler_node"), cfg_(cfg)
{
  // Declare parameters with defaults from cfg_
  cfg_.low_bar_   = this->declare_parameter<double>(ParameterKeys::low_bar,   cfg_.low_bar_);
  cfg_.high_bar_  = this->declare_parameter<double>(ParameterKeys::high_bar,  cfg_.high_bar_);
  cfg_.min_on_s_  = this->declare_parameter<int>(   ParameterKeys::min_on_s,  cfg_.min_on_s_);
  cfg_.min_off_s_ = this->declare_parameter<int>(   ParameterKeys::min_off_s, cfg_.min_off_s_);

  // Telemetry subscription (best-effort, latest only)
  sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "tank_pressure_bar", cfg_.qos,
    [this](const std_msgs::msg::Float32& msg) {
      last_pressure_    = msg.data;
      last_sample_time_ = SteadyClock::now();
    });

  // Service client (reliable req/resp)
  client_ = this->create_client<std_srvs::srv::SetBool>("start_compressor");

  // Periodic evaluation
  timer_ = this->create_wall_timer(500ms, std::bind(&SchedulerNode::tick_, this));

  // Runtime param updates
  param_cb_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params) {
      return this->onParamsSet_(params);
    });

  RCLCPP_INFO(get_logger(),
    "Scheduler ready: low=%.1f, high=%.1f, min_on=%ds, min_off=%ds",
    cfg_.low_bar_, cfg_.high_bar_, cfg_.min_on_s_, cfg_.min_off_s_);
}

void SchedulerNode::tick_() {
  // Require fresh data (<= 2 s old)
  if (!last_pressure_.has_value() ||
      !last_sample_time_.has_value() ||
      (SteadyClock::now() - *last_sample_time_) > 2s) {
    return;  // no decision without telemetry
  }

  const double p = *last_pressure_;

  // Enforce min on/off windows
  const auto now = SteadyClock::now();
  const bool can_turn_on  = !running_ &&
    (!last_state_change_ || (now - *last_state_change_) >= std::chrono::seconds(cfg_.min_off_s_));
  const bool can_turn_off =  running_ &&
    (!last_state_change_ || (now - *last_state_change_) >= std::chrono::seconds(cfg_.min_on_s_));

  // Hysteresis: turn ON if below low_bar; turn OFF if above high_bar
  if (!running_ && p < cfg_.low_bar_ && can_turn_on) {
    requestState_(true, "pressure below low_bar");
  } else if (running_ && p > cfg_.high_bar_ && can_turn_off) {
    requestState_(false, "pressure above high_bar");
  }
}

void SchedulerNode::requestState_(bool want_running, const char* reason) {
  if (!client_->wait_for_service(0s)) {
    RCLCPP_WARN(get_logger(), "start_compressor service not available");
    return;
  }

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = want_running;

  (void)client_->async_send_request(req,
    [this, want_running, reason](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
      auto res = f.get();
      if (res->success) {
        running_ = want_running;
        last_state_change_ = SteadyClock::now();
        RCLCPP_INFO(get_logger(), "Compressor %s (%s)",
                    running_ ? "STARTED" : "STOPPED", reason);
      } else {
        RCLCPP_WARN(get_logger(), "Command rejected: %s", res->message.c_str());
      }
    });
}

rcl_interfaces::msg::SetParametersResult
SchedulerNode::onParamsSet_(const std::vector<rclcpp::Parameter>& params) {
  // Stage copy; only commit if validations pass
  Config new_cfg = cfg_;

  for (const auto& p : params) {
    if (p.get_name() == ParameterKeys::low_bar &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      new_cfg.low_bar_ = p.as_double();

    } else if (p.get_name() == ParameterKeys::high_bar &&
               p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      new_cfg.high_bar_ = p.as_double();

    } else if (p.get_name() == ParameterKeys::min_on_s &&
               p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (p.as_int() <= 0) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false; res.reason = "min_on_s must be > 0";
        return res;
      }
      new_cfg.min_on_s_ = p.as_int();

    } else if (p.get_name() == ParameterKeys::min_off_s &&
               p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (p.as_int() <= 0) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = false; res.reason = "min_off_s must be > 0";
        return res;
      }
      new_cfg.min_off_s_ = p.as_int();
    }
  }

  // Commit
  cfg_ = new_cfg;

  rcl_interfaces::msg::SetParametersResult ok;
  ok.successful = true; ok.reason = "";
  RCLCPP_INFO(get_logger(), "Params updated: low=%.1f high=%.1f min_on=%ds min_off=%ds",
              cfg_.low_bar_, cfg_.high_bar_, cfg_.min_on_s_, cfg_.min_off_s_);
  return ok;
}

} // namespace schn

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  schn::Config cfg;
  auto node = std::make_shared<schn::SchedulerNode>(cfg);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
