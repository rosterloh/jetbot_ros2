#ifndef JETBOT_NODE_DYNAMIXEL_SDK_WRAPPER_HPP_
#define JETBOT_NODE_DYNAMIXEL_SDK_WRAPPER_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <rcutils/logging_macros.h>
//#include <dynamixel_sdk/dynamixel_sdk.h>

#define LOG_INFO RCUTILS_LOG_INFO_NAMED
#define LOG_WARN RCUTILS_LOG_WARN_NAMED
#define LOG_ERROR RCUTILS_LOG_ERROR_NAMED
#define LOG_DEBUG RCUTILS_LOG_DEBUG_NAMED

#define READ_DATA_SIZE 200

namespace jetbot
{
class DynamixelSDKWrapper
{
 public:
  typedef struct
  {
    std::string usb_port;
    uint8_t id;
    uint32_t baud_rate;
    float protocol_version;
  } Device;

  explicit DynamixelSDKWrapper(const Device & device);
  virtual ~DynamixelSDKWrapper();

  template <typename DataByteT>
  DataByteT get_data_from_device(const uint16_t & addr, const uint16_t & length)
  {
    DataByteT data = 0;
    uint8_t * p_data = (uint8_t*)&data;
    uint16_t index = addr - read_memory_.start_addr;

    std::lock_guard<std::mutex> lock(read_data_mutex_);
    switch (length)
    {
      case 1:
        p_data[0] = read_memory_.data[index+0];
        break;

      case 2:
        p_data[0] = read_memory_.data[index+0];
        p_data[1] = read_memory_.data[index+1];
        break;

      case 4:
        p_data[0] = read_memory_.data[index+0];
        p_data[1] = read_memory_.data[index+1];
        p_data[2] = read_memory_.data[index+2];
        p_data[3] = read_memory_.data[index+3];
        break;

      default:
        p_data[0] = read_memory_.data[index+0];
      break;
    }

    return data;
  }

  bool set_data_to_device(
    const uint16_t & addr,
    const uint16_t & length,
    uint8_t * get_data,
    std::string * msg);

  void init_read_memory(const uint16_t & start_addr, const uint16_t & length);
  void read_data_set();

  bool is_connected_to_device();

 private:
  bool init_dynamixel_sdk_handlers();

  bool read_register(
    uint8_t id,
    uint16_t address,
    uint16_t length,
    uint8_t * data_basket,
    const char ** log = NULL);

  bool write_register(
    uint8_t id,
    uint16_t address,
    uint16_t length,
    uint8_t * data,
    const char ** log = NULL);

  //dynamixel::PortHandler * portHandler_;
  //dynamixel::PacketHandler * packetHandler_;

  Device device_;

  uint8_t read_data_[READ_DATA_SIZE] = {0, };
  uint8_t read_data_buffer_[READ_DATA_SIZE] = {0, };

  typedef struct
  {
    uint16_t start_addr;
    uint16_t length;
    uint8_t * data;
  } Memory;

  Memory read_memory_;

  std::mutex sdk_mutex_;
  std::mutex read_data_mutex_;
  std::mutex write_data_mutex_;
};
} // jetbot
#endif // JETBOT_NODE_DYNAMIXEL_SDK_WRAPPER_HPP_