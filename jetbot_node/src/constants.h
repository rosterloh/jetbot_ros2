#include <chrono>
using namespace std::chrono_literals;

namespace jetbot
{
constexpr char SensorStateTopic[] = "sensor_state";
constexpr char JointStateTopic[] = "joint_states";
constexpr char OdomTopic[] = "odom";
constexpr char ScanHalfTopic[] = "scan_half";
constexpr char ScanTopic[] = "scan";
constexpr char ImuTopic[] = "imu";
constexpr char TimeTopic[] = "time_sync";

constexpr auto JointStatePublishPeriodMillis = 33ms;
constexpr auto ScanPublishPeriodMillis = 200ms;
constexpr auto OdometryPublishPeriodMillis = 33ms;
constexpr auto TimeSyncPublishPeriodMillis = 1000ms;

constexpr double WheelRadius = 0.033f;
}