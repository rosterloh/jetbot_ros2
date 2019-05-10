#pragma once

#include <memory>
#include <array>
#include <mutex>

#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace jetbot
{

typedef struct robot
{
    std::array<double, 4> diff_wheels;
    double theta;
} Robot;

class Odometry
{
public:
    Odometry(){};
    virtual ~Odometry(){};

    nav_msgs::msg::Odometry getOdom(const rclcpp::Time now, const double wheel_radius);
    const geometry_msgs::msg::TransformStamped getOdomTf();
    void updateOdomTf(const rclcpp::Time now, const nav_msgs::msg::Odometry odom);
    void updateImu(const sensor_msgs::msg::Imu::SharedPtr imu);
    void updateJointState(const sensor_msgs::msg::JointState::SharedPtr joint_state);

private:
    bool calcOdometry(const rclcpp::Duration duration, const double wheel_radius);

    std::mutex robot_mutex_, tf_mutex_;
    Robot diff_mobile_;

    geometry_msgs::msg::TransformStamped odom_tf_;

    std::array<double, 3> odom_pose_;
    std::array<double, 3> odom_vel_;
};
} // namespace jetbot