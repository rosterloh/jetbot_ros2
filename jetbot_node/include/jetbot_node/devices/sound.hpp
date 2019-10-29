#ifndef JETBOT_NODE_DEVICES_SOUND_HPP_
#define JETBOT_NODE_DEVICES_SOUND_HPP_

#include <jetbot_msgs/srv/sound.hpp>

#include "jetbot_node/devices/devices.hpp"

namespace jetbot
{
namespace devices
{
class Sound : public Devices
{
 public:
  static void request(
    rclcpp::Client<jetbot_msgs::srv::Sound>::SharedPtr client,
    jetbot_msgs::srv::Sound::Request req);

  explicit Sound(
    std::shared_ptr<rclcpp::Node> & nh,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
    const std::string & server_name = "sound");

  void command(const void * request, void * response) override;

 private:
  rclcpp::Service<jetbot_msgs::srv::Sound>::SharedPtr srv_;
};
} // devices
} // jetbot
#endif // JETBOT_NODE_DEVICES_SOUND_HPP_