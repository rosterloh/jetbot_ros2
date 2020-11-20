#pragma once
#include <chrono>
#include "jetbot_msgs/msg/raw_command.hpp"
#include "jetbot_msgs/msg/raw_data.hpp"
#include "jetbot_msgs/msg/raw_motor_command.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace jetbot_msgs;
using namespace std::chrono_literals;

using duration = std::chrono::nanoseconds;
namespace jetbot
{
const uint8_t UART_START_PACKET = 253;
const uint8_t SERIAL_OUT_PACKAGE_LENGTH = 7;
const uint8_t SERIAL_IN_PACKAGE_LENGTH = 5;
const unsigned long BAUDRATE = 57600;

/// Responsible for managing the serial connection and communicating with the
/// jetbot.
class JetBotSerial : public rclcpp::Node
{
public:
  JetBotSerial();

protected:
  std::shared_ptr<const std::array<uint8_t, 3>> motor_efforts_u8;
  std::string serial_port;

  /// Should send messages at this frequency, even if no data is requested.
  duration keepalive_period = 100ms;

  /// If no motor speed is commanded for this long, kill the motors
  duration kill_motors_timeout = 333ms;

  /// How often to poll for a uart message
  duration uart_poll_period = 25ms;

  rclcpp::Publisher<jetbot_msgs::msg::RawData>::SharedPtr pub_raw_data;
  rclcpp::Subscription<jetbot_msgs::msg::RawMotorCommand>::SharedPtr sub_motor_efforts;
  rclcpp::Subscription<jetbot_msgs::msg::RawCommand>::SharedPtr sub_raw_commands;

  void on_raw_command(jetbot_msgs::msg::RawCommand::SharedPtr);
  void read_callback();
  void keepalive_callback();
  void on_kill_motors();
  void on_motor_efforts(jetbot_msgs::msg::RawMotorCommand::SharedPtr msg);
  bool open_serial();
  unsigned int serial_read(std::vector<uint8_t> &inbuf, size_t size);
  unsigned int serial_write(const uint8_t *data, size_t length);
  unsigned int serial_buffer_availible();
  rclcpp::TimerBase::SharedPtr keepalive_timer;
  rclcpp::TimerBase::SharedPtr read_timer;
  rclcpp::TimerBase::SharedPtr kill_motors_timer;

  std::shared_ptr<int> serial_fd_;
};

class jetbotError : public std::runtime_error
{
public:
  jetbotError(const char * msg) : std::runtime_error(msg) {}
  jetbotError(const std::string msg) : std::runtime_error(msg) {}
};
}  // namespace jetbot