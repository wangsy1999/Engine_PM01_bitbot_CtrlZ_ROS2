#pragma once

#include "Workers/NN/CommonLocoInferenceWorker.hpp"
#include "Workers/NN/NetInferenceWorker.h"
#include "Utils/ZenBuffer.hpp"
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace z
{

template<typename SchedulerType, typename InferencePrecision, size_t INPUT_STUCK_LENGTH, size_t JOINT_NUMBER>
class LabInferenceWorker : public CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>
{
public:
    using Base = CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>;
    using MotorValVec = math::Vector<InferencePrecision, JOINT_NUMBER>;
    using ValVec3 = math::Vector<InferencePrecision, 3>;
    using ValVec2     = math::Vector<InferencePrecision, 2>;    
public:
    LabInferenceWorker(SchedulerType* scheduler,
                       const nlohmann::json& Net_cfg,
                       const nlohmann::json& Motor_cfg)
        : Base(scheduler, Net_cfg, Motor_cfg),
          GravityVector({0.0, 0.0, -1.0}),
          hist_ang_vel(INPUT_STUCK_LENGTH),
          hist_proj_grav(INPUT_STUCK_LENGTH),
          hist_cmd3(INPUT_STUCK_LENGTH),
          hist_q(INPUT_STUCK_LENGTH),
          hist_dq(INPUT_STUCK_LENGTH),
          hist_last_action(INPUT_STUCK_LENGTH)
        //   hist_clock(INPUT_STUCK_LENGTH)
    {
        nlohmann::json NetworkCfg = Net_cfg["Network"];
        this->cycle_time = NetworkCfg["Cycle_time"].get<InferencePrecision>();
        this->dt = scheduler->getSpinOnceTime();

        this->PrintSplitLine();
        std::cout << "LabInferenceWorker" << std::endl;
        std::cout << "JOINT_NUMBER = " << JOINT_NUMBER << std::endl;
        std::cout << "Cycle_time   = " << this->cycle_time << std::endl;
        std::cout << "dt           = " << this->dt << std::endl;
        this->PrintSplitLine();

        // output scale
        this->OutputScaleVec = this->ActionScale;

        // ORT tensors
        this->InputOrtTensors__.push_back(this->WarpOrtTensor(InputTensor));
        this->OutputOrtTensors__.push_back(this->WarpOrtTensor(OutputTensor));
    }

    virtual ~LabInferenceWorker() {}

    // ================================================================================
    //                               PreProcess
    // ================================================================================
    void PreProcess() override
    {
        this->start_time = std::chrono::steady_clock::now();

        //---------------------------------------------------------------------
        // 读取状态
        //---------------------------------------------------------------------
        MotorValVec dq;
        this->Scheduler->template GetData<"CurrentMotorVelocity">(dq);

        MotorValVec q;
        this->Scheduler->template GetData<"CurrentMotorPosition">(q);
        q -= this->JointDefaultPos;

        MotorValVec last_act;
        this->Scheduler->template GetData<"NetLastAction">(last_act);

        ValVec3 cmd3;
        this->Scheduler->template GetData<"NetUserCommand3">(cmd3);

        ValVec3 ang_vel;
        this->Scheduler->template GetData<"AngleVelocityValue">(ang_vel);

        ValVec3 ang;
        this->Scheduler->template GetData<"AngleValue">(ang);

        ValVec3 proj_grav = z::ComputeProjectedGravity(ang, this->GravityVector);
        this->Scheduler->template SetData<"NetProjectedGravity">(proj_grav);
            //         ValVec3 Ang;
            // this->Scheduler->template GetData<"AngleValue">(Ang);
            // ValVec3 AngRad = Ang / 180.0 * M_PI;
            // ValVec3 proj_grav = z::ComputeProjectedGravity(AngRad, this->GravityVector);

        size_t t = this->Scheduler->getTimeStamp();

        ang_vel = ang_vel * this->Scales_ang_vel;
        hist_ang_vel.push(ang_vel);

        proj_grav = proj_grav * this->Scales_project_gravity;
        hist_proj_grav.push(proj_grav);

        cmd3 = cmd3 * this->Scales_command3;
        hist_cmd3.push(cmd3);

        q = q * this->Scales_dof_pos;
        hist_q.push(q);

        dq = dq * this->Scales_dof_vel;
        hist_dq.push(dq);

        last_act = last_act * this->Scales_last_action;
        hist_last_action.push(last_act);
        // hist_clock.push(clock);   
        //---------------------------------------------------------------------
        //   拼成最终输入 (45 × INPUT_STUCK_LENGTH)
        //---------------------------------------------------------------------
        math::Vector<InferencePrecision, INPUT_TENSOR_LENGTH> InputVec;
        size_t offset = 0;

        auto copy_stack = [&](auto& buf, size_t dim) {
            for (size_t i = 0; i < INPUT_STUCK_LENGTH; i++) {
                std::copy(buf[i].begin(), buf[i].end(), InputVec.begin() + offset);
                offset += dim;
            }
        };

        copy_stack(hist_ang_vel, 3);
        copy_stack(hist_proj_grav, 3);
        copy_stack(hist_cmd3, 3);
        copy_stack(hist_q, JOINT_NUMBER);
        copy_stack(hist_dq, JOINT_NUMBER);
        copy_stack(hist_last_action, JOINT_NUMBER);
        // copy_stack(hist_clock, 2);
        //---------------------------------------------------------------------
        // clamp
        //---------------------------------------------------------------------
        this->InputTensor.Array() =
            decltype(InputVec)::clamp(InputVec, -this->ClipObservation, this->ClipObservation);
    }

    // ================================================================================
    //                               PostProcess
    // ================================================================================
    void PostProcess() override
    {
        auto last_act = this->OutputTensor.toVector();
        last_act = MotorValVec::clamp(last_act, -this->ClipAction, this->ClipAction);

        this->Scheduler->template SetData<"NetLastAction">(last_act);

        auto scaled = last_act * this->OutputScaleVec + this->JointDefaultPos;
        this->Scheduler->template SetData<"NetScaledAction">(scaled);

        auto clipped = MotorValVec::clamp(scaled, this->JointClipLower, this->JointClipUpper);
        this->Scheduler->template SetData<"TargetMotorPosition">(clipped);

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - this->start_time);
        this->Scheduler->template SetData<"InferenceTime">(
            static_cast<InferencePrecision>(duration.count()));
    }

private:

    // 每帧 45 维 → 共 6 个块
    static constexpr size_t INPUT_TENSOR_LENGTH =
        (3 + 3 + 3 + JOINT_NUMBER + JOINT_NUMBER + JOINT_NUMBER) * INPUT_STUCK_LENGTH;

    static constexpr size_t OUTPUT_TENSOR_LENGTH = JOINT_NUMBER;

    z::math::Tensor<InferencePrecision, 1, INPUT_TENSOR_LENGTH> InputTensor;
    z::math::Tensor<InferencePrecision, 1, OUTPUT_TENSOR_LENGTH> OutputTensor;

    z::math::Vector<InferencePrecision, OUTPUT_TENSOR_LENGTH> OutputScaleVec;

    // 每个观测量独立堆叠
    z::RingBuffer<ValVec3> hist_ang_vel;
    z::RingBuffer<ValVec3> hist_proj_grav;
    z::RingBuffer<ValVec3> hist_cmd3;
    z::RingBuffer<MotorValVec> hist_q;
    z::RingBuffer<MotorValVec> hist_dq;
    z::RingBuffer<MotorValVec> hist_last_action;
    // z::RingBuffer<ValVec2>     hist_clock;
    const ValVec3 GravityVector;

    InferencePrecision cycle_time;
    InferencePrecision dt;

    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point end_time;
};

} // namespace z
