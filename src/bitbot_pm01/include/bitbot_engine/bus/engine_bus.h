#ifndef Engine_BUS_H
#define Engine_BUS_H
#include "bitbot_engine/device/engine_device.hpp"
#include "bitbot_engine/device/engine_imu.h"
#include "bitbot_engine/device/engine_joint.h"
#include "bitbot_engine/kernel/ros_interface.hpp"
#include "bitbot_kernel/bus/bus_manager.hpp"

namespace bitbot {
class EngineBus : public BusManagerTpl<EngineBus, EngineDevice> {
 public:
  EngineBus();
  ~EngineBus();

  void WriteBus();
  void ReadBus();
  // TODO: Is this needed?
  void UpdateDevices();
    void PowerOn();
    void PowerOff();
  inline void SetInterface(const RosInterface::Ptr ros_interface) {
    ros_interface_ = ros_interface;
  }

 protected:
  void doConfigure(const pugi::xml_node& bus_node);
  void doRegisterDevices();

 private:
  RosInterface::Ptr ros_interface_;
    std::vector<EngineDevice*> joint_devices_;
    std::vector<EngineDevice*> imu_devices_;
};
}  // namespace bitbot

#endif  // !Engine_BUS_H
