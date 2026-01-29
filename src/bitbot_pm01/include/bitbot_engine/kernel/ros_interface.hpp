#ifndef ROS_INTERFACE_HPP
#define ROS_INTERFACE_HPP

#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <chrono> // 必须包含

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
    // 1. 初始化标志位
    first_state_received_ = false;

    // publisher: joint command
    // 使用 Best Effort 可靠性较低但延迟低，对于高频控制通常推荐 Reliable + Volatile 或者 BestEffort
    // 如果是用于下发控制，建议保留 Reliable (默认) 或明确指定
    // 这里你用了 depth=3，这是为了防止堆积，是好的。
    joint_cmd_pub_ = this->create_publisher<interface_protocol::msg::JointCommand>(
          "/hardware/joint_command", 3);

    // subscriber: joint state
    joint_state_sub_ = this->create_subscription<interface_protocol::msg::JointState>(
        "/hardware/joint_state", 
        rclcpp::SensorDataQoS(), 
        [this](interface_protocol::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_lock_);
          joint_state_msg_ = *msg;
          first_state_received_ = true; // 标记已收到数据
        });

    // subscriber: imu
    imu_sub_ = this->create_subscription<interface_protocol::msg::ImuInfo>(
        "/hardware/imu_info", 
        rclcpp::SensorDataQoS(),
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
    
    // !!! 初始化 joint_state_msg_ 大小，防止 get 时越界访问
    {
        std::lock_guard<std::mutex> lock(data_lock_);
        joint_state_msg_.position.resize(num_motors, 0.0);
        joint_state_msg_.velocity.resize(num_motors, 0.0);
    }
  }

  static void RunRosSpin(Ptr ptr) {
    ptr->ros_thread_ = std::make_shared<std::thread>([ptr]() { 
        // 捕获异常防止 crash
        try {
            rclcpp::spin(ptr); 
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ptr->get_logger(), "ROS Spin Exception: %s", e.what());
        }
    });
  }

  void PublishJointCommand() {
    joint_cmd_msg_.header.stamp = this->now();
    joint_cmd_pub_->publish(joint_cmd_msg_);
  }

  interface_protocol::msg::JointState GetJointState() {
    std::lock_guard<std::mutex> lock(data_lock_);
    // 这里如果返回的是空数据，外部算法可能会炸
    // 建议配合 WaitingForSystemReady 使用
    return joint_state_msg_;
  }

  interface_protocol::msg::ImuInfo GetImu() {
    std::lock_guard<std::mutex> lock(data_lock_);
    return imu_msg_;
  }

  interface_protocol::msg::JointCommand& Command() {
    return joint_cmd_msg_;
  }

  // 修改：真正的等待就绪
  void WaitingForSystemReady() {
    RCLCPP_INFO(this->get_logger(), "Waiting for first joint state...");
    
    int wait_count = 0;
    while(rclcpp::ok() && !first_state_received_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
        if (wait_count % 10 == 0) {
             RCLCPP_WARN(this->get_logger(), "Still waiting for hardware data...");
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "System Ready. Sensor data received.");
  }

  void UpdateJointState() {
    PublishJointCommand();
  }

  void OnStopSafeCommand() {
    RCLCPP_WARN(this->get_logger(), "PREPARING SAFETY STOP...");

    // 1. 填充安全指令
    // 注意：不要 resize，直接 fill，因为 msg 已经在构造函数分配好内存了
    std::fill(joint_cmd_msg_.position.begin(), joint_cmd_msg_.position.end(), 0.0);
    std::fill(joint_cmd_msg_.velocity.begin(), joint_cmd_msg_.velocity.end(), 0.0);
    std::fill(joint_cmd_msg_.torque.begin(), joint_cmd_msg_.torque.end(), 0.0);
    
    // 确保 vector 不为空再操作
    if (joint_cmd_msg_.stiffness.size() == num_motors) 
        std::fill(joint_cmd_msg_.stiffness.begin(), joint_cmd_msg_.stiffness.end(), 0.0);
    
    if (joint_cmd_msg_.damping.size() == num_motors) 
        std::fill(joint_cmd_msg_.damping.begin(), joint_cmd_msg_.damping.end(), 2.0); // 适当阻尼

    // 2. 冗余发送
    for (int i = 0; i < 5; ++i) {
        if (!rclcpp::ok()) break; // 防止 ctrl+c 后强行发送
        PublishJointCommand();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    RCLCPP_WARN(this->get_logger(), "Safe Stop Command Sent.");
  }

  ~RosInterface() {
    // 析构逻辑优化
    // 1. 如果还在运行，先尝试取消 spin (Context shutdown)
    // 注意：如果这个类不是掌管全局 context 的，慎用 shutdown
    if (rclcpp::ok()) {
        // rclcpp::shutdown();  <-- 建议注释掉这行，除非你确定这是唯一的节点
        // 更好的方式是依靠 shared_ptr 的引用计数释放节点，或者让外部控制 shutdown
    }

    // 2. 等待线程结束
    if (ros_thread_ && ros_thread_->joinable()) {
        // 如果 spin 是死循环，需要外部 signal 来终止，或者在这里 shutdown
        // 既然你之前写了 shutdown，这里我们假设你是主控程序
        rclcpp::shutdown(); 
        ros_thread_->join();
    }
  }

 private:
  std::shared_ptr<std::thread> ros_thread_;
  std::atomic<bool> first_state_received_; // 原子标志位

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