#include "jetbot_node/sensors/imu.hpp"

using namespace jetbot;

sensors::Imu::Imu(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & imu_topic_name,
  const std::string & mag_topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  imu_pub_ = nh->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, this->qos_);
  mag_pub_ = nh->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create imu publisher");
}

void sensors::Imu::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

  imu_msg->header.frame_id = this->frame_id_;
  imu_msg->header.stamp = now;

  imu_msg->orientation.w = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_w.addr,
    extern_control_table.imu_orientation_w.length);

  imu_msg->orientation.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_x.addr,
    extern_control_table.imu_orientation_x.length);

  imu_msg->orientation.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_y.addr,
    extern_control_table.imu_orientation_y.length);

  imu_msg->orientation.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_orientation_z.addr,
    extern_control_table.imu_orientation_z.length);

  imu_msg->angular_velocity.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_angular_velocity_x.addr,
    extern_control_table.imu_angular_velocity_x.length);

  imu_msg->angular_velocity.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_angular_velocity_y.addr,
    extern_control_table.imu_angular_velocity_y.length);

  imu_msg->angular_velocity.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_angular_velocity_z.addr,
    extern_control_table.imu_angular_velocity_z.length);

  imu_msg->linear_acceleration.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_linear_acceleration_x.addr,
    extern_control_table.imu_linear_acceleration_x.length);

  imu_msg->linear_acceleration.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_linear_acceleration_y.addr,
    extern_control_table.imu_linear_acceleration_y.length);

  imu_msg->linear_acceleration.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_linear_acceleration_z.addr,
    extern_control_table.imu_linear_acceleration_z.length);

  auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();

  mag_msg->header.frame_id = this->frame_id_;
  mag_msg->header.stamp = now;

  mag_msg->magnetic_field.x = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_magnetic_x.addr,
    extern_control_table.imu_magnetic_x.length);

  mag_msg->magnetic_field.y = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_magnetic_y.addr,
    extern_control_table.imu_magnetic_y.length);

  mag_msg->magnetic_field.z = dxl_sdk_wrapper->get_data_from_device<float>(
    extern_control_table.imu_magnetic_z.addr,
    extern_control_table.imu_magnetic_z.length);

  imu_pub_->publish(std::move(imu_msg));
  mag_pub_->publish(std::move(mag_msg));
}