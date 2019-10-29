#ifndef JETBOT_NODE_DIFF_DRIVE_CONTROLLER_HPP_
#define JETBOT_NODE_DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "jetbot_node/odometry.hpp"

namespace jetbot
{
class DiffDriveController : public rclcpp::Node
{
 public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController(){};

 private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
} // jetbot
#endif // JETBOT_NODE_DIFF_DRIVE_CONTROLLER_HPP_