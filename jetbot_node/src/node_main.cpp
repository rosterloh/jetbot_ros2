#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "constants.h"
#include "joint_state.h"
#include "odometry.h"

namespace jetbot
{
class Jetbot : public rclcpp::Node
{
public:
    explicit Jetbot(const std::string &node_name)
        : Node(node_name)
    {
        RCLCPP_INFO(get_logger(), "Init Jetbot Node Main");

        node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

        joint_state_ = std::make_shared<JointState>();
        odom_ = std::make_shared<Odometry>();

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(JointStateTopic, rmw_qos_profile_default);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(OdomTopic, rmw_qos_profile_default);
        time_pub_ = this->create_publisher<builtin_interfaces::msg::Time>(TimeTopic, rmw_qos_profile_default);

        joint_state_timer_ = this->create_wall_timer(
            JointStatePublishPeriodMillis,
            [this]() {
                this->joint_state_pub_->publish(this->joint_state_->getJointState(this->now()));
            });

        odom_timer_ = this->create_wall_timer(
            OdometryPublishPeriodMillis,
            [this]() {
                this->odom_->updateJointState(this->joint_state_->getJointState(this->now()));
                this->odom_pub_->publish(this->odom_->getOdom(this->now(), WheelRadius));
                this->tf_broadcaster_->sendTransform(this->odom_->getOdomTf());
            });

        time_timer_ = this->create_wall_timer(
            TimeSyncPublishPeriodMillis,
            [this]() {
                auto time_msg = builtin_interfaces::msg::Time();
                time_msg = this->now();
                this->time_pub_->publish(time_msg);
            });
    }

    virtual ~Jetbot(){};

private:
    rclcpp::Node::SharedPtr node_handle_;

    std::shared_ptr<JointState> joint_state_;
    std::shared_ptr<Odometry> odom_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_pub_;

    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr time_timer_;
};
} // namespace jetbot

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<jetbot::Jetbot>("jetbot_node");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}