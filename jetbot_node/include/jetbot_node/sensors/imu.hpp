#ifndef JETBOT_NODE_SENSORS_IMU_HPP_
#define JETBOT_NODE_SENSORS_IMU_HPP_

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "jetbot_node/sensors/sensors.hpp"

namespace jetbot
{
namespace sensors
{
class Imu : public Sensors
{
 public:
  explicit Imu(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & imu_topic_name = "imu",
    const std::string & mag_topic_name = "magnetic_field",
    const std::string & frame_id = "imu_link");

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper) override;

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
};
} // sensors
} // jetbot
#endif // JETBOT_NODE_SENSORS_IMU_HPP_