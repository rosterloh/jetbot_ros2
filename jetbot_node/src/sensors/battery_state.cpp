#include "jetbot_node/sensors/battery_state.hpp"

using namespace jetbot;

sensors::BatteryState::BatteryState(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name)
: Sensors(nh)
{
  pub_ = nh->create_publisher<sensor_msgs::msg::BatteryState>(topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create battery state publisher");
}

void sensors::BatteryState::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto msg = std::make_unique<sensor_msgs::msg::BatteryState>();

  msg->header.stamp = now;

  msg->design_capacity = 1.8f;

  msg->voltage = 0.01f * dxl_sdk_wrapper->get_data_from_device<int32_t>(
    extern_control_table.battery_voltage.addr,
    extern_control_table.battery_voltage.length);

  msg->percentage = 0.01f * dxl_sdk_wrapper->get_data_from_device<int32_t>(
    extern_control_table.battery_percentage.addr,
    extern_control_table.battery_percentage.length);

  msg->voltage <= 7.0 ? msg->present = false : msg->present = true;

  pub_->publish(std::move(msg));
}