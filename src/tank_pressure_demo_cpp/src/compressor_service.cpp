#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

using SetBool = std_srvs::srv::SetBool;
using Trigger  = std_srvs::srv::Trigger;

class CompressorService : public rclcpp::Node {
public:
  CompressorService() : rclcpp::Node("compressor_service"), running_(false) {
    using std::placeholders::_1; using std::placeholders::_2;

    // Service to reset the compressor state
    srv_start_stop_  = create_service<SetBool>(
      "start_compressor",
      std::bind(&CompressorService::handle_, this, _1, _2));

    // Service to reset interlock
    srv_reset_ = create_service<Trigger>(
      "reset_interlock",
      std::bind(&CompressorService::handleReset_, this, _1, _2));

    // Service to trigger a fault interlock
    srv_fault_ = create_service<Trigger>(
      "fault_interlock",
      std::bind(&CompressorService::handleFault_, this, _1, _2));

    // Logger   
    RCLCPP_INFO(get_logger(), "Service ready: /start_compressor (std_srvs/SetBool)");
  }

private:
  // Handler for start/stop service
  void handle_(const std::shared_ptr<SetBool::Request> req,
               std::shared_ptr<SetBool::Response> res) {
    if (interlockFaulted_) {
      res->success = false;
      res->message = "Interlock fault — cannot change compressor state.";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }
    running_ = req->data;
    res->success = true;
    res->message = running_ ? "Compressor STARTED" : "Compressor STOPPED";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }


  // Reset handler
  void handleReset_(const std::shared_ptr<Trigger::Request> /*req*/,
                    std::shared_ptr<Trigger::Response> res) {
    interlockFaulted_ = false;
    res->success = true;
    res->message = "Interlock reset";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());

  }

  // Fault handler
  void handleFault_(const std::shared_ptr<Trigger::Request> /*req*/,
                    std::shared_ptr<Trigger::Response> res) {
    interlockFaulted_ = true;
    res->success = true;
    res->message = "Interlock faulted";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
}

  // state
  bool running_;
  bool interlockFaulted_{false};

  // service handles
  rclcpp::Service<SetBool>::SharedPtr srv_start_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_fault_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompressorService>());
  rclcpp::shutdown();
  return 0;
}
