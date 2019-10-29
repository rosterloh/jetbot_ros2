#include "jetbot_node/devices/sound.hpp"

using namespace jetbot;

devices::Sound::Sound(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
  const std::string & server_name)
: Devices(nh, dxl_sdk_wrapper)
{
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create sound server");
  srv_ = nh_->create_service<jetbot_msgs::srv::Sound>(
    server_name,
    [this](
      const std::shared_ptr<jetbot_msgs::srv::Sound::Request> request,
      std::shared_ptr<jetbot_msgs::srv::Sound::Response> response) -> void
      {
        this->command(static_cast<void*>(request.get()), static_cast<void*>(response.get()));
      }
    );
}

void devices::Sound::command(const void * request, void * response)
{
  jetbot_msgs::srv::Sound::Request req = *(jetbot_msgs::srv::Sound::Request*)request;
  jetbot_msgs::srv::Sound::Response *res = (jetbot_msgs::srv::Sound::Response*)response;

  res->success = dxl_sdk_wrapper_->set_data_to_device(
    extern_control_table.sound.addr,
    extern_control_table.sound.length,
    (uint8_t*)&req.value,
    &res->message);
}

void devices::Sound::request(
  rclcpp::Client<jetbot_msgs::srv::Sound>::SharedPtr client,
  jetbot_msgs::srv::Sound::Request req)
{
  auto request = std::make_shared<jetbot_msgs::srv::Sound::Request>(req);
  auto result = client->async_send_request(request);
}