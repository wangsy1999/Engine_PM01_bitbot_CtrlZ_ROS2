#ifndef Engine_JOINT_H
#define Engine_JOINT_H

#include "bitbot_engine/device/engine_device.hpp"

namespace bitbot {

enum class EngineJointType {
  NONE = 0,
  SerialPosition,
  SerialTorque,
  ParallelPosition,
  ParallelTorque,
};

class EngineJoint final : public EngineDevice {
 public:
  EngineJoint(const pugi::xml_node& device_node);
  ~EngineJoint();
    void PowerOn();
    void PowerOff();

  inline double GetActualPosition() { return actual_position_; }

  inline double GetActualVelocity() { return actual_velocity_; }

  inline double GetActualTorque() { return actual_torque_; }

  inline void SetTargetPDGains(double p_gain, double d_gain) {
    p_gain_ = p_gain;
    d_gain_ = d_gain;
  }

  inline void SetTargetPosition(double pos) { target_position_ = pos; }

  inline void SetTargetVelocity(double vel) { target_velocity_ = vel; }

  inline void SetTargetTorque(double torque) { target_torque_ = torque; }

 private:
  virtual void Input(const RosInterface::Ptr ros_interface) final;
  virtual void Output(const RosInterface::Ptr ros_interface) final;
  virtual void UpdateModel(const RosInterface::Ptr ros_interface) final;
  virtual void UpdateRuntimeData() final;

 private:
  EngineJointType joint_type_;
  bool enable_ = true;
  bool power_on_ = false;
  unsigned int ros_joint_index_;

  double actual_position_ = 0.0;
  double actual_velocity_ = 0.0;
  double actual_torque_ = 0.0;

  double target_position_ = 0.0;
  double target_velocity_ = 0.0;
  double target_torque_ = 0.0;

  double p_gain_ = 0.0;
  double d_gain_ = 0.0;
};

}  // namespace bitbot

#endif  // !Engine_JOINT_H
