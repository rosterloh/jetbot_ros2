#include <chrono>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class TimeSync : public rclcpp::Node
{
public:
    TimeSync()
        : Node("time_sync")
    {
        RCLCPP_INFO(this->get_logger(), "Init System Time publisher");

        rmw_qos_profile_t time_sync_qos_profile = rmw_qos_profile_default;

        time_sync_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        time_sync_qos_profile.depth = 1;
        time_sync_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        time_sync_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

        time_pub_ = this->create_publisher<builtin_interfaces::msg::Time>("time_sync", time_sync_qos_profile);
        auto timer_callback =
            [this]() -> void {
            auto time_msg = builtin_interfaces::msg::Time();
            time_msg = rclcpp::Clock().now();
            this->time_pub_->publish(time_msg);
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimeSync>());
    rclcpp::shutdown();
    return 0;
}