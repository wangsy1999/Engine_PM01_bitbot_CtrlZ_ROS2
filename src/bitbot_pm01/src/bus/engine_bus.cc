#include "bitbot_engine/bus/engine_bus.h"

namespace bitbot {

EngineBus::EngineBus() {}

EngineBus::~EngineBus() {this->PowerOff();
		// this->WriteBus();
		this->logger_->info("EngineBus shutdown.");}

void EngineBus::doConfigure(const pugi::xml_node& bus_node) {
  CreateDevices(bus_node);
}

void EngineBus::doRegisterDevices() {
  static DeviceRegistrar<EngineDevice, EngineJoint> Engine_joint(
      (uint32_t)EngineDeviceType::Engine_JOINT, "EngineJoint");
  static DeviceRegistrar<EngineDevice, EngineImu> Engine_imu(
      (uint32_t)EngineDeviceType::Engine_IMU, "EngineImu");
}

void EngineBus::WriteBus() {
  for (auto& device : devices_) {
    device->Output(ros_interface_);
  }
  ros_interface_->PublishJointCommand();
}
void EngineBus::PowerOn() {
  // 1. 遍历 devices_ 列表
  for (auto& device : devices_) {
    // 2. 尝试将通用设备指针(EngineDevice*)转换为关节指针(EngineJoint*)
    // 注意：devices_ 里存的是指针，所以这里 device 本身就是指针，直接用 dynamic_cast
    auto joint = dynamic_cast<EngineJoint*>(device);

    // 3. 如果转换成功（指针非空），说明它是关节，执行上电
    if (joint) {
      joint->PowerOn();
    }
  }
}

void EngineBus::PowerOff() {
  // 同样遍历列表
  for (auto& device : devices_) {
    auto joint = dynamic_cast<EngineJoint*>(device);
    if (joint) {
      joint->PowerOff();
    }
  }
}
void EngineBus::ReadBus() {


  for (auto& device : devices_) {
    device->Input(ros_interface_);
  }
}

void EngineBus::UpdateDevices() {
  for (auto& device : devices_) {
    device->UpdateModel(ros_interface_);
  }
}

}  // namespace bitbot
