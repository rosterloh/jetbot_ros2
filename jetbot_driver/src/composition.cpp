#include "composition.hpp"

#include <string>
#include "jetbot.hpp"
// #include "jetbot_serial.hpp"
using namespace jetbot;

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  // auto jetbot_serial_node = std::make_shared<JetBotSerial>();
  // executor.add_node(jetbot_serial_node);

  auto jetbot_node = std::make_shared<JetBot>();
  executor.add_node(jetbot_node);

  executor.spin();
  return 0;
}