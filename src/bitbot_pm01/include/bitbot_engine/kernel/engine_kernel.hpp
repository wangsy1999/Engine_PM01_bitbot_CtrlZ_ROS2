#ifndef Engine_KERNEL_HPP
#define Engine_KERNEL_HPP

#include <thread>

#include "bitbot_engine/kernel/ros_interface.hpp"   // 你已经替换成新版 ros_interface.hpp
#include "bitbot_kernel/kernel/kernel.hpp"

// 如果你已经没有 EngineBus，那这里要换成你的 RosBus / JointBus。
// 我先保持 EngineBus 不动（因为我不知道你 Bus 层怎么写的），
// 但你需要把 EngineBus 的实现也换成“纯 ROS topic 的设备更新”。
#include "bitbot_engine/bus/engine_bus.h"

enum class EngineKernelState : uint32_t {
  POWER_ON = 100,
  POWER_ON_FINISH,
};

enum class EngineKernelEvent : uint32_t {
  POWER_ON = 100,
  POWER_ON_FINISH,
};

namespace bitbot {

template <typename UserData, CTString... cts>
class EngineKernel : public KernelTpl<EngineKernel<UserData, cts...>,
                                       EngineBus, UserData, cts...> {
 public:
  EngineKernel(std::string config_file)
      : KernelTpl<EngineKernel<UserData, cts...>, EngineBus, UserData, cts...>(
            config_file) {
    pugi::xml_node const& bitbot_node = this->parser_->GetBitbotNode();
    (void)bitbot_node;

    ros_interface_ = std::make_shared<RosInterface>();
    RosInterface::RunRosSpin(ros_interface_);

    this->KernelRegisterEvent(
        "power_on", static_cast<EventId>(EngineKernelState::POWER_ON),
        [this](EventValue, UserData&) -> std::optional<StateId> {
          this->logger_->info("joints power on");
          this->busmanager_.PowerOn();
          return static_cast<StateId>(EngineKernelState::POWER_ON);
        },
        false);

    this->KernelRegisterEvent(
        "power_on_finish",
        static_cast<EventId>(EngineKernelState::POWER_ON_FINISH),
        [this](EventValue, UserData&) -> std::optional<StateId> {
          this->logger_->info("joints power on finish");
          return static_cast<StateId>(EngineKernelState::POWER_ON_FINISH);
        },
        false);

    this->InjectEventsToState(
        static_cast<StateId>(KernelState::IDLE),
        {static_cast<EventId>(EngineKernelEvent::POWER_ON)});

    this->KernelRegisterState(
        "power on", static_cast<StateId>(EngineKernelState::POWER_ON),
        [this](const bitbot::KernelInterface& kernel, auto& extra_data,
               UserData& user_data) {
          (void)kernel;
          (void)extra_data;
          (void)user_data;

          // 替代原先 EnterCustomMode：等待 joint_states 就绪
          ros_interface_->WaitingForSystemReady();

          // 可选：在这里发一次“上电后初始化命令”
          // 例如：发一个零位/保持姿态/使能标志（取决于你的下游）
          // ros_interface_->PublishJointCommand();

          this->EmitEvent(static_cast<EventId>(EngineKernelEvent::POWER_ON_FINISH), 0);
        },
        {static_cast<EventId>(EngineKernelEvent::POWER_ON_FINISH)});

    this->KernelRegisterState(
        "power on finish",
        static_cast<StateId>(EngineKernelState::POWER_ON_FINISH),
        [this](const bitbot::KernelInterface& kernel, auto& extra_data,
               UserData& user_data) {
          (void)kernel;
          (void)extra_data;
          (void)user_data;
        },
        {static_cast<EventId>(KernelEvent::START)});

    this->busmanager_.SetInterface(ros_interface_);
  }

  ~EngineKernel() = default;

 public:
  void doStart() {
    RCLCPP_INFO(rclcpp::get_logger("bitbot kernel"), "Kernel started.");
          this->busmanager_.ReadBus(); 


      // ============================================================
      // [修复步骤 2]: 写入总线 (发布命令)
      // 注意：你在 ros_interface.hpp 里定义的 UpdateJointState 其实是 PublishCommand
      // 建议这里调用 WriteBus，然后在 Bus 层的 WriteBus 里调用 Output
      // ============================================================
      // this->busmanager_.WriteBus(); 
  }

  void doRun() {
    RCLCPP_DEBUG(rclcpp::get_logger("bitbot_ros_interface"), "once");

    auto this_time = std::chrono::high_resolution_clock::now();
    auto last_time = this_time;

    ros_interface_->WaitingForSystemReady();
    // this->busmanager_.UpdateDevices();

    while (!this->kernel_config_data_.stop_flag) {
      this_time = std::chrono::high_resolution_clock::now();

      // 计算周期时间 (代码不变)
      this->kernel_runtime_data_.periods_count++;
      this->kernel_runtime_data_.period =
          std::chrono::duration_cast<std::chrono::nanoseconds>(this_time - last_time)
              .count() /
          1e6;
      last_time = this_time;

      // ============================================================
      // [修复步骤 1]: 必须在这里读取总线！
      // 这会触发 Engine_bus.cc -> ReadBus -> Engine_joint.cc -> Input
      // 从而把 ros_interface 里的数据更新到 actual_position_ 等变量中
      // ============================================================

      
      // 如果你的 WriteBus 没有调用 publish，保留下面这行；
      // 如果 EngineBus::WriteBus 内部调用了 device->Output，且 device->Output 更新了 cmd 消息，
      // 那么你需要确保最后有一个地方调用了 ros_interface_->PublishJointCommand()。
      // ros_interface_->UpdateJointState();

      // 2) kernel tasks
      this->HandleEvents();
      this->KernelLoopTask();
      this->KernelPrivateLoopEndTask();

      // 3) loop timing
      auto end_time = std::chrono::high_resolution_clock::now();
      this->kernel_runtime_data_.process_time =
          std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - this_time)
              .count() /
          1e6;

      auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::high_resolution_clock::now() - this_time)
                         .count();
      if (elapsed < 2000) [[likely]] {
        std::this_thread::sleep_for(std::chrono::microseconds(2000 - elapsed));
      } else {
        RCLCPP_WARN(rclcpp::get_logger("bitbot_ros_interface"),
                    "Kernel loop over time: %ld us", elapsed);
      }
    }
    ros_interface_->OnStopSafeCommand();
  }

 private:
  RosInterface::Ptr ros_interface_;
};

}  // namespace bitbot

#endif  // Engine_KERNEL_HPP
