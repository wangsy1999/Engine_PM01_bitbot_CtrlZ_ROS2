/**
 * @file HumanoidGymInferenceWorker.hpp
 * @author Zishun Zhou
 * @brief
 *
 * @date 2025-03-10
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "CommonLocoInferenceWorker.hpp"
#include "NetInferenceWorker.h"
#include "Utils/ZenBuffer.hpp"
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace z
{
    /**
     * @brief HumanoidGymInferenceWorker 类型是一个人形机器人推理工人类型，该类实现了HumanoidGym网络兼容的推理功能。
     * @details HumanoidGymInferenceWorker 类型是一个人形机器人推理工人类型，该类实现了HumanoidGym网络兼容的推理功能。
     * HumanoidGym参见[https://github.com/roboterax/humanoid-gym](https://github.com/roboterax/humanoid-gym)
     *
     * @details config.json配置文件示例：
     * {
     *  "Scheduler": {
     *    "dt": 0.001 //调度器的时间步长 1ms
     *  },
     *  "Workers": {
     *     "NN": {
     *        "NetWork":{
     *           "Cycle_time": 0.63 //步频周期
     *       }
     *     }
     *   }
     * }
     *
     * @tparam SchedulerType 调度器类型
     * @tparam InferencePrecision 推理精度，用户可以通过这个参数来指定推理的精度，比如可以指定为float或者double
     * @tparam INPUT_STUCK_LENGTH HumanoidGym网络的Actor输入堆叠长度
     * @tparam JOINT_NUMBER 关节数量
     */
    template<typename SchedulerType, typename InferencePrecision, size_t INPUT_STUCK_LENGTH, size_t JOINT_NUMBER>
    class HumanLabStandInferenceWorker : public CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>
    {
    public:
        using Base = CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>;
        using MotorValVec = math::Vector<InferencePrecision, JOINT_NUMBER>;
        using ValVec3 = math::Vector<InferencePrecision, 3>;

    public:HumanLabStandInferenceWorker(SchedulerType* scheduler, const nlohmann::json& Net_cfg, const nlohmann::json& Motor_cfg)
            :CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>(scheduler, Net_cfg, Motor_cfg),
            GravityVector({ 0.0,0.0,-1.0 }),
            HistoryInputBuffer(INPUT_STUCK_LENGTH)
        {
            //read cfg
            nlohmann::json InferenceCfg = Net_cfg["Inference"];
            nlohmann::json NetworkCfg = Net_cfg["Network"];
            this->cycle_time = NetworkCfg["Cycle_time"].get<InferencePrecision>();
            this->dt = scheduler->getSpinOnceTime();

            this->PrintSplitLine();
            std::cout << "LabInferenceWorker" << std::endl;
            std::cout << "JOINT_NUMBER=" << JOINT_NUMBER << std::endl;
            std::cout << "Cycle_time=" << this->cycle_time << std::endl;
            std::cout << "dt=" << this->dt << std::endl;
            this->PrintSplitLine();

            //concatenate all scales
            auto clock_scales = math::Vector<InferencePrecision, 2>::ones();
            this->InputScaleVec = math::cat(
                this->Scales_ang_vel,
                this->Scales_project_gravity,
                this->Scales_command3,
                clock_scales,
                this->Scales_dof_pos,
                this->Scales_dof_vel,
                this->Scales_last_action
            );
            this->OutputScaleVec = this->ActionScale;

            //warp input tensor
            this->InputOrtTensors__.push_back(this->WarpOrtTensor(InputTensor));
            this->OutputOrtTensors__.push_back(this->WarpOrtTensor(OutputTensor));
        }

        /**
         * @brief 析构函数
         *
         */
        virtual ~HumanLabStandInferenceWorker()
        {

        }
        // === 单事件控制接口 ===
        // 开/关相位（true=运行节奏；false=冻结并回默认）
        void SetPhaseActive(bool active) {
            if (active && !phase_active_) {
                // 上升沿：从0相位重新开始
                phase_start_t_ = this->Scheduler->getTimeStamp();
                // 或者使用内部累加器：phase_acc_ = 0;
            }
            phase_active_ = active;
        }

    // 可选：切换（若你用按钮触发）
    void TogglePhaseActive() { SetPhaseActive(!phase_active_); }

        /**
         * @brief 推理前的准备工作,主要是将数据从数据总线中读取出来，并将数据缩放到合适的范围
         * 构造堆叠的输入数据，并准备好输入张量。
         *
         */
        void PreProcess() override
        {
            this->start_time = std::chrono::steady_clock::now();

            MotorValVec CurrentMotorVel;
            this->Scheduler->template GetData<"CurrentMotorVelocity">(CurrentMotorVel);

            MotorValVec CurrentMotorPos;
            this->Scheduler->template GetData<"CurrentMotorPosition">(CurrentMotorPos);
            CurrentMotorPos -= this->JointDefaultPos;

            MotorValVec LastAction;
            this->Scheduler->template GetData<"NetLastAction">(LastAction);

            ValVec3 UserCmd3;
            this->Scheduler->template GetData<"NetUserCommand3">(UserCmd3);

            ValVec3 LinVel;
            this->Scheduler->template GetData<"LinearVelocityValue">(LinVel);

            ValVec3 AngVel;
            this->Scheduler->template GetData<"AngleVelocityValue">(AngVel);

            ValVec3 Ang;
            this->Scheduler->template GetData<"AngleValue">(Ang);
            ValVec3 ProjectedGravity = z::ComputeProjectedGravity(Ang, this->GravityVector);

            size_t t = this->Scheduler->getTimeStamp();

            // ==== 单事件控制相位 ====
            constexpr InferencePrecision alpha_back = (InferencePrecision)0.3; // 回默认的平滑系数

            if (phase_active_) {
                // 注：打开时 SetPhaseActive(true) 已记录了 phase_start_t_
                InferencePrecision phase = this->dt * static_cast<InferencePrecision>(t - phase_start_t_) / this->cycle_time;
                // InferencePrecision pw = std::fmod(phase, (InferencePrecision)1.0);
                // if (pw < 0) pw += 1.0;
                clock_sin = std::sin(phase * 2.0 * M_PI);
                clock_cos = std::cos(phase * 2.0 * M_PI);
            } else {
                // 关闭：回到默认 (0, 1)
                constexpr InferencePrecision target_sin = (InferencePrecision)0.0;
                constexpr InferencePrecision target_cos = (InferencePrecision)1.0;
                clock_sin += alpha_back * (target_sin - clock_sin);
                clock_cos += alpha_back * (target_cos - clock_cos);
}



            // InferencePrecision phase = this->dt * static_cast<InferencePrecision>(t) / this->cycle_time;
            // InferencePrecision clock_sin = std::sin(phase * 2 * M_PI);
            // InferencePrecision clock_cos = std::cos(phase * 2 * M_PI);
            ValVec3 Cmd3Effective;
            if (phase_active_) {
                Cmd3Effective = UserCmd3;                      // 正常使用外部指令
            } else {
                Cmd3Effective = ValVec3::zeros();              // 冻结 → 指令强制 0
            }
            // 可选：把“生效后的指令”写回数据总线便于观测
            // this->Scheduler->template SetData<"NetUserCommand3Effective">(Cmd3Effective);
            // ==========================================

            z::math::Vector<InferencePrecision, 2> ClockVector = { clock_sin, clock_cos };
            this->Scheduler->template SetData<"NetClockVector">(ClockVector);

            auto SingleInputVecScaled = math::cat(
                AngVel,
                ProjectedGravity,
                Cmd3Effective,        // 使用“生效后的”命令
                ClockVector,
                CurrentMotorPos,
                CurrentMotorVel,
                LastAction
            ) * this->InputScaleVec;

            this->HistoryInputBuffer.push(SingleInputVecScaled);


            math::Vector<InferencePrecision, INPUT_TENSOR_LENGTH> InputVec;
            for (size_t i = 0; i < INPUT_STUCK_LENGTH; i++)
            {
                std::copy(this->HistoryInputBuffer[i].begin(), this->HistoryInputBuffer[i].end(), InputVec.begin() + i * INPUT_TENSOR_LENGTH_UNIT);
            }

            this->InputTensor.Array() = decltype(InputVec)::clamp(InputVec, -this->ClipObservation, this->ClipObservation);
        }

        /**
         * @brief 推理后的处理工作,主要是将推理的结果从数据总线中读取出来，并将数据缩放到合适的范围
         *
         */
        void PostProcess() override
        {
            auto LastAction = this->OutputTensor.toVector();
            auto ClipedLastAction = MotorValVec::clamp(LastAction, -this->ClipAction, this->ClipAction);
            this->Scheduler->template SetData<"NetLastAction">(ClipedLastAction);

            auto ScaledAction = ClipedLastAction * this->OutputScaleVec + this->JointDefaultPos;
            this->Scheduler->template SetData<"NetScaledAction">(ScaledAction);

            auto clipedAction = MotorValVec::clamp(ScaledAction, this->JointClipLower, this->JointClipUpper);
            this->Scheduler->template SetData<"TargetMotorPosition">(clipedAction);

            this->end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(this->end_time - this->start_time);
            InferencePrecision inference_time = static_cast<InferencePrecision>(duration.count());
            this->Scheduler->template SetData<"InferenceTime">(inference_time);
        }

    private:
        //clock; usercmd;q;dq;act;angle vel;euler xyz;
        static constexpr size_t INPUT_TENSOR_LENGTH_UNIT = 3 + JOINT_NUMBER + JOINT_NUMBER + JOINT_NUMBER + 3 + 3 + 2;
        static constexpr size_t INPUT_TENSOR_LENGTH = INPUT_TENSOR_LENGTH_UNIT * INPUT_STUCK_LENGTH;
        //joint number
        static constexpr size_t OUTPUT_TENSOR_LENGTH = JOINT_NUMBER;
        // 类成员变量（需要你添加到类中）：
        InferencePrecision phase_acc = 0.0;        // 累积的 phase 时间
        bool phase_active = false;                 // 当前是否处于激活状态
        int phase_start_t = 0;                     // 激活时刻的时间步（单位是 t）
        InferencePrecision clock_sin = 0.0;
        InferencePrecision clock_cos = 1.0;
        //input tensor
        z::math::Tensor<InferencePrecision, 1, INPUT_TENSOR_LENGTH> InputTensor;
        z::math::Vector<InferencePrecision, INPUT_TENSOR_LENGTH_UNIT> InputScaleVec;
        z::RingBuffer<z::math::Vector<InferencePrecision, INPUT_TENSOR_LENGTH_UNIT>> HistoryInputBuffer;

        //output tensor
        z::math::Tensor<InferencePrecision, 1, OUTPUT_TENSOR_LENGTH> OutputTensor;
        z::math::Vector<InferencePrecision, OUTPUT_TENSOR_LENGTH> OutputScaleVec;

        /// @brief 重力向量{0,0,-1}
        const ValVec3 GravityVector;
// 由“单事件”驱动的相位状态
        bool phase_active_ = false;     // 是否运行节奏
        size_t phase_start_t_ = 0;      // 打开时的起点时间戳（用于基于 t 的相位）
        // 如果你更喜欢独立时钟，改用：InferencePrecision phase_acc_ = 0;

        //cycle time and dt
        InferencePrecision cycle_time;
        InferencePrecision dt;

        //compute time
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point end_time;
    };
};

