#ifndef JETBOT_NODE_SENSORS_SENSOR_STATE_HPP_
#define JETBOT_NODE_SENSORS_SENSOR_STATE_HPP_

#include <jetbot_msgs/msg/sensor_state.hpp>

#include "jetbot_node/sensors/sensors.hpp"

namespace jetbot
{
namespace sensors
{
class SensorState : public Sensors
{
 public:
  explicit SensorState(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & topic_name = "sensor_state",
    const bool & bumper_forward = false,
    const bool & bumper_backward = false,
    const bool & cliff = false,
    const bool & sonar = false,
    const bool & illumination = false);

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

 private:
  rclcpp::Publisher<jetbot_msgs::msg::SensorState>::SharedPtr pub_;

  bool bumper_forward_;
  bool bumper_backward_;
  bool cliff_;
  bool sonar_;
  bool illumination_;
};
} // sensors
} // jetbot
#endif // JETBOT_NODE_SENSORS_SENSOR_STATE_HPP_