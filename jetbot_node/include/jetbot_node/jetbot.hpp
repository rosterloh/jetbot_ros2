#ifndef JETBOT_NODE_JETBOT_HPP_
#define JETBOT_NODE_JETBOT_HPP_

#include <array>
#include <chrono>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <jetbot_msgs/msg/sensor_state.hpp>

#include "jetbot_node/control_table.hpp"
#include "jetbot_node/dynamixel_sdk_wrapper.hpp"
#include "jetbot_node/odometry.hpp"

#include "jetbot_node/devices/devices.hpp"
#include "jetbot_node/devices/motor_power.hpp"
#include "jetbot_node/devices/reset.hpp"
#include "jetbot_node/devices/sound.hpp"

#include "jetbot_node/sensors/battery_state.hpp"
#include "jetbot_node/sensors/imu.hpp"
#include "jetbot_node/sensors/joint_state.hpp"
#include "jetbot_node/sensors/sensor_state.hpp"
#include "jetbot_node/sensors/sensors.hpp"

namespace jetbot
{
extern const ControlTable extern_control_table;
class JetBot : public rclcpp::Node
{
 public:
  typedef struct
  {
    float separation;
    float radius;
  } Wheels;

  typedef struct
  {
    float profile_acceleration_constant;
    float profile_acceleration;
  } Motors;

  explicit JetBot(const std::string & usb_port);
  virtual ~JetBot(){};

  Wheels * get_wheels();
  Motors * get_motors();

 private:
  void init_dynamixel_sdk_wrapper(const std::string & usb_port);
  void check_device_status();

  void add_sensors();
  void add_devices();
  void add_motors();
  void add_wheels();

  void run();

  void publish_timer(const std::chrono::milliseconds timeout);
  void heartbeat_timer(const std::chrono::milliseconds timeout);

  void cmd_vel_callback();
  void parameter_event_callback();

  Wheels wheels_;
  Motors motors_;

  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  std::list<sensors::Sensors*> sensors_;
  std::map<std::string, devices::Devices*> devices_;

  std::unique_ptr<Odometry> odom_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};
} // jetbot
#endif // JETBOT_NODE_JETBOT_HPP_