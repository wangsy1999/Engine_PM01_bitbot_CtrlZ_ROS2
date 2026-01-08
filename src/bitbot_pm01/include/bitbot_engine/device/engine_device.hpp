#ifndef Engine_DEVICE_HPP
#define Engine_DEVICE_HPP

#include "bitbot_engine/kernel/ros_interface.hpp"
#include "bitbot_kernel/device/device.hpp"

namespace bitbot {

enum class EngineDeviceType : uint32_t {
  Engine_DEVICE = 12000,
  Engine_JOINT,
  Engine_IMU,
};

class EngineDevice : public Device {
 public:
  EngineDevice(const pugi::xml_node& device_node) : Device(device_node) {}
  ~EngineDevice() = default;

  // Method to w/r Engine
  virtual void UpdateModel(const RosInterface::Ptr ros_interface) = 0;
  virtual void Input(const RosInterface::Ptr ros_interface) = 0;
  virtual void Output(const RosInterface::Ptr ros_interface) = 0;

  // Inherited from Device base calss
  virtual void UpdateRuntimeData() = 0;

 private:
};
}  // namespace bitbot

#endif  // !Engine_DEVICE_HPP
