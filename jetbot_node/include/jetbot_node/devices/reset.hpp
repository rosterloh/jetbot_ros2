#ifndef JETBOT_NODE_DEVICES_RESET_HPP_
#define JETBOT_NODE_DEVICES_RESET_HPP_

#include <std_srvs/srv/trigger.hpp>

#include "jetbot_node/devices/devices.hpp"

namespace jetbot
{
namespace devices
{
class Reset : public Devices
{
 public:
  static void request(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
    std_srvs::srv::Trigger::Request req);

  explicit Reset(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "reset");

  void command(const void * request, void * response) override;

 private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};
} // devices
} // jetbot
#endif // JETBOT_NODE_DEVICES_RESET_HPP_