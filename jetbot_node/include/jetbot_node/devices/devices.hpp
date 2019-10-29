#ifndef JETBOT_NODE_DEVICES_DEVICES_HPP_
#define JETBOT_NODE_DEVICES_DEVICES_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "jetbot_node/control_table.hpp"
#include "jetbot_node/dynamixel_sdk_wrapper.hpp"

namespace jetbot
{
extern const ControlTable extern_control_table;
namespace devices
{
class Devices{
 public:
  explicit Devices(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
  : nh_(nh),
    dxl_sdk_wrapper_(dxl_sdk_wrapper)
  {
  }

  virtual void command(const void * request, void * response) = 0;

 protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::ServicesQoS());
};
} // devices
} // jetbot
#endif // JETBOT_NODE_DEVICES_DEVICES_HPP_