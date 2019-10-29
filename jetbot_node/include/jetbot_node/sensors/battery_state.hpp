#ifndef JETBOT_NODE_SENSORS_BATTERY_STATE_HPP_
#define JETBOT_NODE_SENSORS_BATTERY_STATE_HPP_

#include <sensor_msgs/msg/battery_state.hpp>

#include "jetbot_node/sensors/sensors.hpp"

namespace jetbot
{
namespace sensors
{
class BatteryState : public Sensors
{
 public:
  explicit BatteryState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "battery_state");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

 private:
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
};
} // sensors
} // jetbot
#endif // JETBOT_NODE_SENSORS_BATTERY_STATE_HPP_