#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class TankPressureSubscriber : public rclcpp::Node {
public:
  TankPressureSubscriber() : rclcpp::Node("tank_pressure_sub_cpp") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "tank_pressure_bar", qos,
      [this](const std_msgs::msg::Float32 & msg){
        RCLCPP_INFO(this->get_logger(), "Tank pressure = %.1f bar", msg.data);
      });
  }
private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TankPressureSubscriber>());
  rclcpp::shutdown();
  return 0;
}
