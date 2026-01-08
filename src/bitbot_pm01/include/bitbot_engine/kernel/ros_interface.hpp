#ifndef ROS_INTERFACE_HPP
#define ROS_INTERFACE_HPP

#include <mutex>
#include <thread>
#include <atomic>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "rclcpp/qos.hpp"
#include "interface_protocol/msg/parallel_parser_type.hpp"
namespace bitbot {

constexpr size_t num_motors = 24;

class RosInterface : public rclcpp::Node {
 public:
  using Ptr = std::shared_ptr<RosInterface>;

  RosInterface() : rclcpp::Node("bitbot_ros_interface") {
    // publisher: joint command
joint_cmd_pub_ =
      this->create_publisher<interface_protocol::msg::JointCommand>(
          "/hardware/joint_command", 1);

    // subscriber: joint state
joint_state_sub_ =
    this->create_subscription<interface_protocol::msg::JointState>(
        "/hardware/joint_state", 
        rclcpp::SensorDataQoS(), // <--- 强制使用 SensorData (Best Effort, Volatile)
        [this](interface_protocol::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_lock_);
          joint_state_msg_ = *msg;
        });

// 修改 IMU 订阅同理
imu_sub_ =
    this->create_subscription<interface_protocol::msg::ImuInfo>(
        "/hardware/imu_info", 
        rclcpp::SensorDataQoS(), // <--- 强制使用 SensorData
        [this](interface_protocol::msg::ImuInfo::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_lock_);
          imu_msg_ = *msg;
        });
    
    // init command msg
    joint_cmd_msg_.position.resize(num_motors, 0.0);
    joint_cmd_msg_.velocity.resize(num_motors, 0.0);
    joint_cmd_msg_.torque.resize(num_motors, 0.0);
    joint_cmd_msg_.feed_forward_torque.resize(num_motors, 0.0);
    joint_cmd_msg_.stiffness.resize(num_motors, 0.0); 
    joint_cmd_msg_.damping.resize(num_motors, 0.0);
    joint_cmd_msg_.parallel_parser_type = interface_protocol::msg::ParallelParserType::RL_PARSER;
  }

  static void RunRosSpin(Ptr ptr) {
    ptr->ros_thread_ =
        std::make_shared<std::thread>([ptr]() { rclcpp::spin(ptr); });
  }

  void PublishJointCommand() {
    joint_cmd_msg_.header.stamp = this->now();
    joint_cmd_pub_->publish(joint_cmd_msg_);
  }

  interface_protocol::msg::JointState GetJointState() {
    std::lock_guard<std::mutex> lock(data_lock_);
    return joint_state_msg_;
  }

  interface_protocol::msg::ImuInfo GetImu() {
    std::lock_guard<std::mutex> lock(data_lock_);
    return imu_msg_;
  }

  interface_protocol::msg::JointCommand& Command() {
    return joint_cmd_msg_;
  }



  // 1. 补充等待系统就绪函数
  // 如果不需要特殊等待逻辑，留空或简单休眠即可
  void WaitingForSystemReady() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(this->get_logger(), "System Ready.");
  }

  // 2. 补充 UpdateJointState 函数
  // Kernel 期望用这个名字来发布命令，我们直接调用已有的 PublishJointCommand
  void UpdateJointState() {
    PublishJointCommand();
  }

  // 3. 补充安全停止函数
  // 当系统退出或急停时调用，这里发送零指令
void OnStopSafeCommand() {
    RCLCPP_WARN(this->get_logger(), "PREPARING SAFETY STOP...");

    // 1. 设置安全数据 (Kp=0, Kd=2~5)
    std::fill(joint_cmd_msg_.position.begin(), joint_cmd_msg_.position.end(), 0.0);
    std::fill(joint_cmd_msg_.velocity.begin(), joint_cmd_msg_.velocity.end(), 0.0);
    std::fill(joint_cmd_msg_.torque.begin(), joint_cmd_msg_.torque.end(), 0.0);
    
    // 关键：刚度归零，给一点阻尼防摔
    if (!joint_cmd_msg_.stiffness.empty()) 
        std::fill(joint_cmd_msg_.stiffness.begin(), joint_cmd_msg_.stiffness.end(), 0.0);
    if (!joint_cmd_msg_.damping.empty()) 
        std::fill(joint_cmd_msg_.damping.begin(), joint_cmd_msg_.damping.end(), 2.0);

    // 2. [关键修改] 连续发送 5 次，每次间隔 10ms
    // 为什么要发多次？防止丢包。
    // 为什么要 sleep？给 DDS 网络层一点时间把数据把缓冲区推出去。
    for (int i = 0; i < 5; ++i) {
        PublishJointCommand();
        
        // 这里的 sleep 非常关键！它让 CPU 有机会把网络包发出去
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    RCLCPP_WARN(this->get_logger(), "Safe Stop Command Sent (5 times redundancy).");
  }

  ~RosInterface() {
    // 1. 告诉 ROS 停止 spin
    if (rclcpp::ok()) {
        rclcpp::shutdown(); 
    }

    // 2. 等待线程结束 (Join)
    if (ros_thread_ && ros_thread_->joinable()) {
        ros_thread_->join();
    }
  }

 private:
  std::shared_ptr<std::thread> ros_thread_;

  rclcpp::Publisher<interface_protocol::msg::JointCommand>::SharedPtr joint_cmd_pub_;
  rclcpp::Subscription<interface_protocol::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<interface_protocol::msg::ImuInfo>::SharedPtr imu_sub_;

  std::mutex data_lock_;
  interface_protocol::msg::JointState joint_state_msg_;
  interface_protocol::msg::ImuInfo imu_msg_;

  interface_protocol::msg::JointCommand joint_cmd_msg_;
};

}  // namespace bitbot

#endif
