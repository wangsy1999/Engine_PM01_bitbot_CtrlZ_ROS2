#include "bitbot_engine/device/engine_imu.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace bitbot {
Eigen::Vector3d CalcRollPitchYawFromRotationMatrix(const Eigen::Matrix3d& R) {
  const Eigen::Quaterniond quaternion(R);
  using std::abs;
  using std::atan2;
  using std::sqrt;

  const double R22 = R(2, 2);
  const double R21 = R(2, 1);
  const double R10 = R(1, 0);
  const double R00 = R(0, 0);
  const double Rsum = sqrt((R22 * R22 + R21 * R21 + R10 * R10 + R00 * R00) / 2);
  const double R20 = R(2, 0);
  const double q2 = atan2(-R20, Rsum);

  // Calculate q1 and q3 from Steps 2-6 (documented above).
  const double e0 = quaternion.w(), e1 = quaternion.x();
  const double e2 = quaternion.y(), e3 = quaternion.z();
  const double yA = e1 + e3, xA = e0 - e2;
  const double yB = e3 - e1, xB = e0 + e2;
  const double epsilon = Eigen::NumTraits<double>::epsilon();
  const auto isSingularA = abs(yA) <= epsilon && abs(xA) <= epsilon;
  const auto isSingularB = abs(yB) <= epsilon && abs(xB) <= epsilon;
  const double zA = (isSingularA ? double{0.0} : atan2(yA, xA));
  const double zB = (isSingularB ? double{0.0} : atan2(yB, xB));
  double q1 = zA - zB;  // First angle in rotation sequence.
  double q3 = zA + zB;  // Third angle in rotation sequence.

  q1 = (q1 > M_PI ? q1 - 2 * M_PI : q1);
  q1 = (q1 < -M_PI ? q1 + 2 * M_PI : q1);
  q3 = (q3 > M_PI ? q3 - 2 * M_PI : q3);
  q3 = (q3 < -M_PI ? q3 + 2 * M_PI : q3);

  // Return in (roll-pitch-yaw) order
  return Eigen::Vector3<double>(q1, q2, q3);
}
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

  Eigen::Vector3d rpy = CalcRollPitchYawFromRotationMatrix(q.toRotationMatrix());
  
  // roll_  = imu_msg.rpy.x;
  // pitch_ = imu_msg.rpy.y;
  // yaw_   = imu_msg.rpy.z;
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


