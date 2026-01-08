/**
 * @file user_func.cpp
 * @author Zishun Zhou
 * @brief
 * @date 2025-03-10
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "user_func.h"

#include <chrono>
#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>
#include <memory>
#include <thread>
#include <iostream> // std::cout
#include <nlohmann/json.hpp>
#include <fstream>
#include "types.hpp"


void ConfigFunc(const KernelBus& bus, UserData& d)
{
    //读取json配置文件,并初始化各个worker
    nlohmann::json cfg_root;
    nlohmann::json cfg_workers;
    {
        //NOTE: 注意将配置文件路径修改为自己的路径
        std::string path = PROJECT_ROOT_DIR + std::string("/config.json");
        std::ifstream cfg_file(path);
        cfg_root = nlohmann::json::parse(cfg_file, nullptr, true, true);
        cfg_workers = cfg_root["Workers"];
    }

    //创建调度器
    d.TaskScheduler = new SchedulerType(cfg_root["Scheduler"]);


    // 初始化各个worker
    enum {
        left_hip_yaw_joint=0,  left_hip_roll_joint, left_hip_pitch_joint, left_knee_joint, left_ankle_pitch_joint, left_ankle_roll_joint,
        right_hip_yaw_joint,   right_hip_roll_joint, right_hip_pitch_joint, right_knee_joint, right_ankle_pitch_joint,right_ankle_roll_joint,
        J12_WAIST_YAW,
        J13_SHOULDER_PITCH_L, J14_SHOULDER_ROLL_L, J15_SHOULDER_YAW_L, J16_ELBOW_PITCH_L, J17_ELBOW_YAW_L,
        J18_SHOULDER_PITCH_R, J19_SHOULDER_ROLL_R, J20_SHOULDER_YAW_R, J21_ELBOW_PITCH_R, J22_ELBOW_YAW_R,
        J23_HEAD_YAW

    };



    //初始化各个worker
    d.ImuWorker = new ImuWorkerType(d.TaskScheduler, bus.GetDevice<DeviceImu>(24).value(), cfg_workers["ImuProcess"]);
    d.MotorWorker = new MotorWorkerType(d.TaskScheduler, cfg_workers["MotorControl"], {\
            bus.GetDevice<DeviceJoint>(left_hip_yaw_joint).value(),
            
            bus.GetDevice<DeviceJoint>(right_hip_yaw_joint).value(),
           
            bus.GetDevice<DeviceJoint>(left_hip_roll_joint  ).value(),
            bus.GetDevice<DeviceJoint>(right_hip_roll_joint  ).value(),

            bus.GetDevice<DeviceJoint>(left_hip_pitch_joint    ).value(),
            bus.GetDevice<DeviceJoint>(right_hip_pitch_joint    ).value(),
            bus.GetDevice<DeviceJoint>(left_knee_joint  ).value(),
            bus.GetDevice<DeviceJoint>(right_knee_joint  ).value(),
            bus.GetDevice<DeviceJoint>(left_ankle_pitch_joint  ).value(),
            bus.GetDevice<DeviceJoint>(right_ankle_pitch_joint  ).value(),
            bus.GetDevice<DeviceJoint>(left_ankle_roll_joint  ).value(),
            bus.GetDevice<DeviceJoint>(right_ankle_roll_joint  ).value(),
            bus.GetDevice<DeviceJoint>(J12_WAIST_YAW).value(),
            
            bus.GetDevice<DeviceJoint>(J13_SHOULDER_PITCH_L).value(),
           
            bus.GetDevice<DeviceJoint>(J18_SHOULDER_PITCH_R  ).value(),
            bus.GetDevice<DeviceJoint>(J14_SHOULDER_ROLL_L  ).value(),

            bus.GetDevice<DeviceJoint>(J19_SHOULDER_ROLL_R    ).value(),
            bus.GetDevice<DeviceJoint>(J15_SHOULDER_YAW_L    ).value(),
            bus.GetDevice<DeviceJoint>(J20_SHOULDER_YAW_R  ).value(),
            bus.GetDevice<DeviceJoint>(J16_ELBOW_PITCH_L  ).value(),
            bus.GetDevice<DeviceJoint>(J21_ELBOW_PITCH_R  ).value(),
            bus.GetDevice<DeviceJoint>(J17_ELBOW_YAW_L  ).value(),
            bus.GetDevice<DeviceJoint>(J22_ELBOW_YAW_R  ).value(),
            bus.GetDevice<DeviceJoint>(J23_HEAD_YAW  ).value(),

        }
    );

    // d.MotorPDWorker = new MotorPDWorkerType(d.TaskScheduler, cfg_workers["MotorPDLoop"]);

    d.Logger = new LoggerWorkerType(d.TaskScheduler, cfg_workers["AsyncLogger"]);
    d.CommanderWorker = new CmdWorkerType(d.TaskScheduler, cfg_workers["Commander"]);

    // 创建主任务列表，并添加worker
    d.TaskScheduler->CreateTaskList("MainTask", 1, true);
    d.TaskScheduler->AddWorkers("MainTask",
                                {d.ImuWorker,
                                //  d.MotorPDWorker,
                                 d.MotorWorker,
                                 });



    // 创建复位任务列表，并添加worker，设置复位任务频率为主任务频率的1/10
    d.MotorResetWorker = new MotorResetWorkerType(d.TaskScheduler, cfg_workers["MotorControl"], cfg_workers["ResetPosition"]);

    d.TaskScheduler->CreateTaskList("ResetTask", 10);
    d.TaskScheduler->AddWorker("ResetTask", d.MotorResetWorker);


    // 开始调度器
    d.TaskScheduler->Start();


}

void FinishFunc(UserData &d)
{
    // 删除worker
    delete d.TaskScheduler;
    delete d.ImuWorker;
    delete d.MotorWorker;
    // delete d.MotorPDWorker;
    delete d.Logger;
    delete d.MotorResetWorker;
    delete d.CommanderWorker;
}

std::optional<bitbot::StateId> EventInitPose(bitbot::EventValue value, UserData& d)
{
    if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    {
        d.MotorResetWorker->StartReset(); //开始复位
        d.TaskScheduler->EnableTaskList("ResetTask"); //在复位任务列表中启用复位任务
        return static_cast<bitbot::StateId>(States::PF2InitPose);
    }
    return std::optional<bitbot::StateId>();
}



std::optional<bitbot::StateId> EventSystemTest(bitbot::EventValue value,
    UserData& user_data)
{
    if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    {
        return static_cast<bitbot::StateId>(States::PF2SystemTest);
    }
    return std::optional<bitbot::StateId>();
}



void StateWaiting(const bitbot::KernelInterface& kernel,
    Kernel::ExtraData& extra_data, UserData& d)
{
    // 1. 定义静态变量，程序生命周期内只初始化一次
    static bool is_initialized = false;

    // 2. 必须先运行调度器，确保 CurrentMotorPosition 更新了数据
    d.TaskScheduler->SpinOnce();

    // // 3. 如果是第一次进入，执行锁定逻辑
    // if (!is_initialized) {
    //     // 将当前位置设为目标位置（只执行一次，实现位置锁定）
    //     d.MotorWorker->SetCurrentPositionAsTargetPosition();
        
    //     // 标记为已完成
    //     is_initialized = true;
        
    //     std::cout << "[StateWaiting] Position Locked (Static Init)." << std::endl;
    // }
}

void StateSystemTest(const bitbot::KernelInterface& kernel,
    Kernel::ExtraData& extra_data, UserData& user_data)
{
}



void StateJointInitPose(const bitbot::KernelInterface& kernel,
    Kernel::ExtraData& extra_data, UserData& user_data)
    {   //static int flag = 0;

    user_data.TaskScheduler->SpinOnce();
}
