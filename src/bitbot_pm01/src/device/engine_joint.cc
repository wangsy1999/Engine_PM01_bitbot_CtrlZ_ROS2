#include "bitbot_engine/device/engine_joint.h"

namespace bitbot {

EngineJoint::EngineJoint(const pugi::xml_node& device_node)
    : EngineDevice(device_node) {
  basic_type_ = (uint32_t)BasicDeviceType::MOTOR;
  type_ = (uint32_t)EngineDeviceType::Engine_JOINT;

  monitor_header_.headers = {
    "joint_type",
      "target_position", "p_gain", "d_gain",
      "target_torque", "actual_position", "actual_velocity", "actual_torque",
  };
  monitor_data_.resize(monitor_header_.headers.size());



  ConfigParser::ParseAttribute2b(enable_, device_node.attribute("enable"));
  ConfigParser::ParseAttribute2ui(ros_joint_index_,
                                  device_node.attribute("ros_joint_index"));

  // std::string mode_str;
  // ConfigParser::ParseAttribute2s(mode_str, device_node.attribute("mode"));
    double p_gain, d_gain;
    ConfigParser::ParseAttribute2d(p_gain, device_node.attribute("kp"));
    ConfigParser::ParseAttribute2d(d_gain, device_node.attribute("kd"));
    this->p_gain_ = static_cast<float>(p_gain);
    this->d_gain_ = static_cast<float>(d_gain);
}

EngineJoint::~EngineJoint() {}
  void EngineJoint::PowerOn()
  {
    this->target_position_ = this->actual_position_;
    this->target_velocity_ = 0;
    this->target_torque_ = 0;
    this->power_on_ = true;
  }

  void EngineJoint::PowerOff()
  {
    this->power_on_ = false;
    this->target_position_ = this->actual_position_;
    this->target_velocity_ = 0;
    this->target_torque_ = 0;
  }
void EngineJoint::Input(const RosInterface::Ptr ros_interface) {
  // 1. 获取对象（注意不是指针）
  auto js = ros_interface->GetJointState(); 

  // 2. 删除这行：对象永远不为空，不能用 !js 判断
  // if (!js) return;

  const size_t idx = ros_joint_index_;
  // 3. 改为点操作符 (.)，并增加 empty 检查
  if (js.position.empty() || idx >= js.position.size()) return;

  // 4. 所有 -> 改为 .
  actual_position_ = js.position[idx];
  actual_velocity_ = (idx < js.velocity.size()) ? js.velocity[idx] : 0.0;
  actual_torque_   = (idx < js.torque.size())   ? js.torque[idx]   : 0.0;
}
void EngineJoint::Output(const RosInterface::Ptr ros_interface) {
  if (!enable_) return;
// if (this->id_ == 0) {
// // 使用 ros_interface 的时钟（记得解引用 *）
// RCLCPP_INFO_THROTTLE(rclcpp::get_logger("joint_debug"), *ros_interface->get_clock(), 1000,
//     "Joint 0: PowerOn=%d, Enable=%d, TargetPos=%f", 
//     (int)power_on_, (int)enable_, target_position_);
//     }
  // 修改这里：调用 Command()
  auto& cmd = ros_interface->Command(); 
  
  const size_t idx = ros_joint_index_;
  
  // 确保 cmd 已经 resize (RosInterface 构造函数里做了，但检查一下更安全)
  if (idx >= cmd.position.size()) return;
  if (power_on_ && enable_) {

    cmd.position[idx] = target_position_;
    cmd.velocity[idx] = 0;
    cmd.torque[idx] = 0;
    // 注意：ROS标准JointState没有 stiffness/damping，除非你自定义了消息
    // 如果报错说没有 stiffness，请注释掉下面两行
    cmd.stiffness[idx] = p_gain_; 
    cmd.damping[idx]   = d_gain_;
  }
  else{

    cmd.position[idx] = 0.0;
    cmd.velocity[idx] = 0.0;
    cmd.torque[idx] = 0.0;
    // 注意：ROS标准JointState没有 stiffness/damping，除非你自定义了消息
    // 如果报错说没有 stiffness，请注释掉下面两行
    cmd.stiffness[idx] = 0.0; 
    cmd.damping[idx]   = 0.0;
  }
}

void EngineJoint::UpdateRuntimeData() {
  constexpr double rad2deg = 180.0 / M_PI;

  monitor_data_[0] = (power_on_ && enable_) ? 1.0 : 0.0;
  monitor_data_[1] = rad2deg * target_position_;
  monitor_data_[2] = p_gain_;
  monitor_data_[3] = d_gain_;
  monitor_data_[4] = target_torque_;
  monitor_data_[5] = rad2deg * actual_position_;
  monitor_data_[6] = actual_velocity_;
  monitor_data_[7] = actual_torque_;
}
void EngineJoint::UpdateModel(const RosInterface::Ptr) {}

}  // namespace bitbot
