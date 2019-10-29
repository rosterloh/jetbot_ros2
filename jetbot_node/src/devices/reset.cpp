#include "jetbot_node/devices/reset.hpp"

using namespace jetbot;

devices::Reset::Reset(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
  const std::string & server_name)
: Devices(nh, dxl_sdk_wrapper)
{
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create reset server");
  srv_ = nh_->create_service<std_srvs::srv::Trigger>(
    server_name,
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
      {
        this->command(static_cast<void*>(request.get()), static_cast<void*>(response.get()));
      }
    );
}

void devices::Reset::command(const void * request, void * response)
{
  (void) request;

  std_srvs::srv::Trigger::Response *res = (std_srvs::srv::Trigger::Response*)response;

  uint8_t reset = 1;

  res->success = dxl_sdk_wrapper_->set_data_to_device(
    extern_control_table.imu_re_calibration.addr,
    extern_control_table.imu_re_calibration.length,
    &reset,
    &res->message);

  RCLCPP_INFO(nh_->get_logger(), "Start Calibration of Gyro");
  rclcpp::sleep_for(std::chrono::seconds(5));
  RCLCPP_INFO(nh_->get_logger(), "Calibration End");
}

void devices::Reset::request(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
  std_srvs::srv::Trigger::Request req)
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>(req);
  auto result = client->async_send_request(request);
}