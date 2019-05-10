#pragma once

#include <memory>
#include <array>
#include <mutex>

#include "rclcpp/time.hpp"

//#include "turtlebot3_msgs/msg/sensor_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace jetbot
{
class JointState
{
public:
    JointState(){};
    virtual ~JointState(){};

    sensor_msgs::msg::JointState::SharedPtr getJointState(const rclcpp::Time now);
    //void updateRadianFromTick(const turtlebot3_msgs::msg::SensorState::SharedPtr state);

private:
    std::array<double, 2> last_rad_ = {0.0, 0.0};
    std::array<int32_t, 2> last_diff_tick_ = {0, 0};

    std::mutex mutex_;
};
} // namespace jetbot