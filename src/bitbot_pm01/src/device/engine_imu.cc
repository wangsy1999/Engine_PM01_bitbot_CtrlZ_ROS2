#include "bitbot_engine/device/engine_imu.h"

#include <Eigen/Geometry>

namespace bitbot {

EngineImu::EngineImu(pugi::xml_node const& device_node)
    : EngineDevice(device_node) {
  basic_type_ = (uint32_t)BasicDeviceType::IMU;
  type_ = (uint32_t)EngineDeviceType::Engine_IMU;

  monitor_header_.headers = {"roll",  "pitch",  "yaw",    "acc_x", "acc_y",
                             "acc_z", "gyro_x", "gyro_y", "gyro_z"};
  monitor_data_.resize(monitor_header_.headers.size());
}

EngineImu::~EngineImu() {}

void EngineImu::Input(const RosInterface::Ptr ros_interface) {
  auto imu_msg = ros_interface->GetImu();

  // ===== quaternion -> rpy =====
  Eigen::Quaterniond q(
      imu_msg.quaternion.w,
      imu_msg.quaternion.x,
      imu_msg.quaternion.y,
      imu_msg.quaternion.z);

  Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
  roll_  = rpy[0];
  pitch_ = rpy[1];
  yaw_   = rpy[2];

  // ===== linear acceleration =====
  acc_x_ = imu_msg.linear_acceleration.x;
  acc_y_ = imu_msg.linear_acceleration.y;
  acc_z_ = imu_msg.linear_acceleration.z;

  // ===== angular velocity =====
  gyro_x_ = imu_msg.angular_velocity.x;
  gyro_y_ = imu_msg.angular_velocity.y;
  gyro_z_ = imu_msg.angular_velocity.z;
}

void EngineImu::Output(const RosInterface::Ptr) {}

void EngineImu::UpdateRuntimeData() {
  constexpr double rad2deg = 180.0 / M_PI;

  monitor_data_[0] = rad2deg * roll_;
  monitor_data_[1] = rad2deg * pitch_;
  monitor_data_[2] = rad2deg * yaw_;
  monitor_data_[3] = acc_x_;
  monitor_data_[4] = acc_y_;
  monitor_data_[5] = acc_z_;
  monitor_data_[6] = gyro_x_;
  monitor_data_[7] = gyro_y_;
  monitor_data_[8] = gyro_z_;
}

void EngineImu::UpdateModel(const RosInterface::Ptr ros_interface) {}

}  // namespace bitbot
