/**
 * @file main.cpp
 * @author zishun zhou (zhouzishun@mail.zzshub.cn)
 * @brief
 *
 * @date 2025-03-04
 *
 * @copyright Copyright (c) 2025
 */

// # include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "user_func.h"


int main(int argc, char const* argv[])
{
      rclcpp::init(argc, argv);
    //NOTE: 注意将配置文件路径修改为自己的路径
    std::string cfg_path = PROJECT_ROOT_DIR + std::string("/bitbot_engine.xml");
    Kernel kernel(cfg_path);

    kernel.RegisterConfigFunc(ConfigFunc);
    kernel.RegisterFinishFunc(FinishFunc);

    // 注册 Event
    kernel.RegisterEvent("system_test",
        static_cast<bitbot::EventId>(Events::SystemTest),
        &EventSystemTest);

    kernel.RegisterEvent("init_pose",
        static_cast<bitbot::EventId>(Events::InitPose),
        &EventInitPose);
    // kernel.RegisterEvent("policy_run",
    //     static_cast<bitbot::EventId>(Events::PolicyRun),
    //     &EventPolicyRun);


    // 注册 State
    kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::Waiting),
        &StateWaiting,
        { static_cast<bitbot::EventId>(Events::SystemTest), (Events::InitPose) });

    kernel.RegisterState("SystemTest", static_cast<bitbot::StateId>(States::PF2SystemTest), &StateSystemTest, {});

    kernel.RegisterState("init_pose",
        static_cast<bitbot::StateId>(States::PF2InitPose),
        &StateJointInitPose,
        { });


    // kernel.RegisterState("policy_run",
    //     static_cast<bitbot::StateId>(States::PF2PolicyRun),
    //     &StatePolicyRun, { static_cast<bitbot::EventId>(Events::VeloxDecrease), static_cast<bitbot::EventId>(Events::VeloxIncrease),
    //     static_cast<bitbot::EventId>(Events::VeloyDecrease), static_cast<bitbot::EventId>(Events::VeloyIncrease),
    //     static_cast<bitbot::EventId>(Events::VeloYawDecrease),static_cast<bitbot::EventId>(Events::VeloYawIncrease),
    //     static_cast<bitbot::EventId>(Events::JoystickXChange), static_cast<bitbot::EventId>(Events::JoystickYChange),
    //     static_cast<bitbot::EventId>(Events::JoystickYawChange)});

    kernel.SetFirstState(static_cast<bitbot::StateId>(States::Waiting));
    kernel.Run(); // Run the kernel
      rclcpp::shutdown();
    return 0;
}