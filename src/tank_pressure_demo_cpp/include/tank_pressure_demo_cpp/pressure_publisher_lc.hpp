#pragma once
#include <chrono>
#include <random>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <std_msgs/msg/float32.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <lifecycle_msgs/msg/state.hpp> 

namespace tpd {

class PressurePublisherLC : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit PressurePublisherLC(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // lifecycle hooks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure  (const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate   (const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate (const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup    (const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown   (const rclcpp_lifecycle::State &);

  // publish loop
  void tick();

  // params
  std::string topic_{"tank_pressure"};
  double mean_bar_{96.0};
  double jitter_bar_{0.5};
  int period_ms_{100};
  double sp1_warn_bar_{200.0};
  double sp2_error_bar_{300.0};

  // infra
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::random_device rd_;
  std::mt19937 gen_{rd_()};
  std::uniform_real_distribution<float> dist_{-0.5f, 0.5f};

  // diagnostics
  diagnostic_updater::Updater diag_{this};
  double last_pressure_bar_{0.0};
  void diagnosticsCb_(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

} // namespace tpd
