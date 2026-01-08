/**
 * @file user_func.h
 * @author Zishun Zhou
 * @brief
 * @date 2025-03-10
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "types.hpp"
#include "include/bitbot_engine/kernel/engine_kernel.hpp"
enum Events
{
    InitPose = 1001,
    SystemTest,

};

enum class States : bitbot::StateId
{
    Waiting = 1001,
    PF2InitPose,
    PF2SystemTest,
};

struct UserData
{
    SchedulerType *TaskScheduler;
    ImuWorkerType *ImuWorker;
    MotorWorkerType *MotorWorker;
    MotorPDWorkerType *MotorPDWorker;
    LoggerWorkerType *Logger;
    // EraxLikeInferWorkerType* NetInferWorker;
    // UnitreeRlGymInferWorkerType *NetInferWorker;
    // HumanoidGymLSTMInferWorkerType *NetInferWorker;
    // HumanoidGymInferWorkerType *NetInferWorker;
    MotorResetWorkerType *MotorResetWorker;
    CmdWorkerType *CommanderWorker;
    // NOTE: REMEMBER TO DELETE THESE POINTERS IN FinishFunc
};

using Kernel = bitbot::EngineKernel<UserData>;
using KernelBus = bitbot::EngineBus;
void ConfigFunc(const KernelBus& bus, UserData& d);
void FinishFunc(UserData& d);
std::optional<bitbot::StateId> EventInitPose(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventSystemTest(bitbot::EventValue value, UserData& user_data);





void StateWaiting(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);
void StateJointInitPose(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);
void StateSystemTest(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);
