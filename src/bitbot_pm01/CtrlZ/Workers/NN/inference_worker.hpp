#pragma once
#include <Workers/NN/CommonLocoInferenceWorker.hpp>
#include <Workers/NN/NetInferenceWorker.h>
#include "Utils/ZenBuffer.hpp"
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
namespace bhr_j
{
    using namespace z;

    template<typename SchedulerType, typename InferencePrecision, size_t INPUT_STUCK_LENGTH, size_t JOINT_NUMBER>
    class HumanLabInferenceWorker : public CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>
    {
    public:
        using Base = CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>;
        using MotorValVec = math::Vector<InferencePrecision, JOINT_NUMBER>;
        using ValVec3 = math::Vector<InferencePrecision, 3>;

    public:
    
        HumanLabInferenceWorker(SchedulerType* scheduler, const nlohmann::json& Net_cfg, const nlohmann::json& Motor_cfg)
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
            std::cout << "EraxInferenceWorker" << std::endl;
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
        virtual ~HumanLabInferenceWorker()
        {

        }

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
            ValVec3 AngRad = Ang/180.0*3.1415926;
            ValVec3 ProjectedGravity = z::ComputeProjectedGravity(AngRad, this->GravityVector);

            size_t t = this->Scheduler->getTimeStamp();



            // 每次控制循环调用：
            InferencePrecision cmd_norm = std::sqrt(
                UserCmd3[0] * UserCmd3[0] +
                UserCmd3[1] * UserCmd3[1] +
                UserCmd3[2] * UserCmd3[2]);

            // 当满足门限条件时，开始累积 phase
            if (cmd_norm >= static_cast<InferencePrecision>(0.5)) {
                if (!phase_active) {
                    phase_active = true;
                    phase_start_t = t;  // 当前激活的起始时刻
                }

            // 计算 phase，相对起点进行累加
            InferencePrecision phase = this->dt * (t - phase_start_t) / this->cycle_time;
            InferencePrecision phase_wrapped = std::fmod(phase, 1.0);  // phase ∈ [0,1)

            clock_sin = std::sin(phase_wrapped * 2.0 * M_PI);
            clock_cos = std::cos(phase_wrapped * 2.0 * M_PI);
                }
            else {
                // 停止节奏累加，但保持当前 sin/cos，不跳变
                phase_active = false;
                // 保持 clock_sin / clock_cos 不变，或者可选设置为常数
                clock_sin = 0.0;
                clock_cos = 1.0;
            }


            z::math::Vector<InferencePrecision, 2> ClockVector = { clock_sin, clock_cos };
            this->Scheduler->template SetData<"NetClockVector">(ClockVector);

            auto SingleInputVecScaled = math::cat(
                AngVel,
                ProjectedGravity, // TODO need project gravety
                UserCmd3,
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
        static constexpr size_t INPUT_TENSOR_LENGTH_UNIT = 3 + JOINT_NUMBER + JOINT_NUMBER + JOINT_NUMBER + 3 + 3+2;
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

        //cycle time and dt
        InferencePrecision cycle_time;
        InferencePrecision dt;

        //compute time
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point end_time;
    };
};

