/**
 * @file UnitreeRlGymInferenceWorker.hpp
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
     * @brief UnitreeRlGymInferenceWorker 类型是一个人形机器人推理工人类型，该类实现了Unitree_rl_gym网络兼容的推理功能。
     * @details UnitreeRlGymInferenceWorker 类型是一个人形机器人推理工人类型，该类实现了Unitree_rl_gym网络兼容的推理功能。
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
    class UnitreeRlGymInferenceWorker : public CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>
    {
    public:
        using Base = CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>;
        using MotorValVec = math::Vector<InferencePrecision, JOINT_NUMBER>;
        using ValVec3 = math::Vector<InferencePrecision, 3>;

    public:
        /**
         * @brief 构造一个UnitreeRlGymInferenceWorker类型
         *
         * @param scheduler 调度器的指针
         * @param cfg 配置文件
         */
        UnitreeRlGymInferenceWorker(SchedulerType* scheduler, const nlohmann::json& cfg)
            :CommonLocoInferenceWorker<SchedulerType, InferencePrecision, JOINT_NUMBER>(scheduler, cfg),
            GravityVector({ 0.0,0.0,-1.0 }),
            HistoryInputBuffer(INPUT_STUCK_LENGTH)
        {
            //read cfg
            nlohmann::json InferenceCfg = cfg["Workers"]["NN"]["Inference"];
            nlohmann::json NetworkCfg = cfg["Workers"]["NN"]["Network"];
            this->cycle_time = NetworkCfg["Cycle_time"].get<InferencePrecision>();
            this->dt = cfg["Scheduler"]["dt"].get<InferencePrecision>();

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
                this->Scales_dof_pos,
                this->Scales_dof_vel,
                this->Scales_last_action,
                clock_scales
            );
            this->OutputScaleVec = this->ActionScale;

            //warp input tensor
            this->InputOrtTensors__.push_back(this->WarpOrtTensor(InputTensor));
            this->InputOrtTensors__.push_back(this->WarpOrtTensor(InputHiddenStateTensor));
            this->InputOrtTensors__.push_back(this->WarpOrtTensor(InputCellStateTensor));
            
            this->OutputOrtTensors__.push_back(this->WarpOrtTensor(OutputTensor));
            this->OutputOrtTensors__.push_back(this->WarpOrtTensor(OutputHiddenStateTensor));
            this->OutputOrtTensors__.push_back(this->WarpOrtTensor(OutputCellStateTensor));
        }

        /**
         * @brief 析构函数
         *
         */
        virtual ~UnitreeRlGymInferenceWorker()
        {

        }

// 实际顺序 B → 训练顺序 A
        static constexpr std::array<size_t, JOINT_NUMBER> B_to_A_map = {0, 2, 4, 6, 8, 10, 1, 3, 5, 7, 9, 11};

        // 🧩 模型顺序 → 实际顺序（A → B）
        static constexpr std::array<size_t, JOINT_NUMBER> A_to_B_map = []{
            std::array<size_t, JOINT_NUMBER> map{};
            for (size_t i = 0; i < JOINT_NUMBER; ++i) {
                map[B_to_A_map[i]] = i;
            }
            return map;
        }();

        // 通用 remap 工具
        template<typename VecType, size_t N>
        static VecType Remap(const VecType& input, const std::array<size_t, N>& map) {
            VecType output;
            for (size_t i = 0; i < N; ++i)
                output[i] = input[map[i]];
            return output;
        }
        template<typename T>
        T Remap(const T& input, const std::array<size_t, JOINT_NUMBER>& map) {
            T output;
            for (size_t i = 0; i < JOINT_NUMBER; ++i)
                output[map[i]] = input[i];
            return output;
        }

        /**
         * @brief 推理前的准备工作,主要是将数据从数据总线中读取出来，并将数据缩放到合适的范围
         * 构造堆叠的输入数据，并准备好输入张量。
         *
         */
        void PreProcess() override
        {
            this->start_time = std::chrono::steady_clock::now();

            MotorValVec CurrentMotorVel_raw;
            this->Scheduler->template GetData<"CurrentMotorVelocity">(CurrentMotorVel_raw);
            MotorValVec CurrentMotorVel = RemapBtoA<MotorValVec, JOINT_NUMBER>(CurrentMotorVel_raw, B_to_A_map);

            MotorValVec CurrentMotorPos_raw;
            this->Scheduler->template GetData<"CurrentMotorPosition">(CurrentMotorPos_raw);
            CurrentMotorPos_raw -= this->JointDefaultPos;
            MotorValVec CurrentMotorPos = RemapBtoA<MotorValVec, JOINT_NUMBER>(CurrentMotorPos_raw, B_to_A_map);

            MotorValVec LastAction_raw;
            this->Scheduler->template GetData<"NetLastAction">(LastAction_raw);
            MotorValVec LastAction = RemapBtoA<MotorValVec, JOINT_NUMBER>(LastAction_raw, B_to_A_map);


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
            InferencePrecision phase = this->dt * static_cast<InferencePrecision>(t) / this->cycle_time; // TODO might overflow after a looooong time.
            InferencePrecision clock_sin = std::sin(phase * 2 * M_PI);
            InferencePrecision clock_cos = std::cos(phase * 2 * M_PI);
            z::math::Vector<InferencePrecision, 2> ClockVector = { clock_sin, clock_cos };
            this->Scheduler->template SetData<"NetClockVector">(ClockVector);

            auto SingleInputVecScaled = math::cat(
                AngVel,
                ProjectedGravity, // TODO need project gravety
                UserCmd3,
                CurrentMotorPos,
                CurrentMotorVel,
                LastAction,
                ClockVector
            ) * this->InputScaleVec;

            this->HistoryInputBuffer.push(SingleInputVecScaled);


            math::Vector<InferencePrecision, INPUT_TENSOR_LENGTH> InputVec;
            InputVec = SingleInputVecScaled;
            // for (size_t i = 0; i < INPUT_STUCK_LENGTH; i++)
            // {
            //     std::copy(this->HistoryInputBuffer[i].begin(), this->HistoryInputBuffer[i].end(), InputVec.begin() + i * INPUT_TENSOR_LENGTH_UNIT);
            // }

            this->InputTensor.Array() = decltype(InputVec)::clamp(InputVec, -this->ClipObservation, this->ClipObservation);

        }

        /**
         * @brief 推理后的处理工作,主要是将推理的结果从数据总线中读取出来，并将数据缩放到合适的范围
         *
         */
        void PostProcess() override
        {
            auto ModelOutput = this->OutputTensor.toVector();  // 模型输出 (A顺序)

            auto ClipedModelOutput = MotorValVec::clamp(ModelOutput, -this->ClipAction, this->ClipAction);

// 🌀 模型输出 → 实际顺序（A → B）
            MotorValVec ClipedLastAction = Remap<MotorValVec, JOINT_NUMBER>(ClipedModelOutput, A_to_B_map);

            this->Scheduler->template SetData<"NetLastAction">(ClipedLastAction);

            auto ScaledAction = ClipedLastAction * this->OutputScaleVec + this->JointDefaultPos;
            this->Scheduler->template SetData<"NetScaledAction">(ScaledAction);

            auto clipedAction = MotorValVec::clamp(ScaledAction, this->JointClipLower, this->JointClipUpper);
            this->Scheduler->template SetData<"TargetMotorPosition">(clipedAction);

            this->end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(this->end_time - this->start_time);
            InferencePrecision inference_time = static_cast<InferencePrecision>(duration.count());
            this->Scheduler->template SetData<"InferenceTime">(inference_time);
            InputHiddenStateTensor.Array() = OutputHiddenStateTensor.Array();
            InputCellStateTensor.Array() = OutputCellStateTensor.Array();
        }

    private:
        //clock; usercmd;q;dq;act;angle vel;euler xyz;
        static constexpr size_t INPUT_TENSOR_LENGTH_UNIT = 2 + 3 + JOINT_NUMBER + JOINT_NUMBER + JOINT_NUMBER + 3 + 3;
        static constexpr size_t INPUT_TENSOR_LENGTH = INPUT_TENSOR_LENGTH_UNIT * INPUT_STUCK_LENGTH;
        //joint number
        static constexpr size_t OUTPUT_TENSOR_LENGTH = JOINT_NUMBER;
        static constexpr size_t HIDDEN_STATE_LENGTH = 64;

        //input tensor
        z::math::Tensor<InferencePrecision, 1, INPUT_TENSOR_LENGTH> InputTensor;
        z::math::Tensor<InferencePrecision, 1, 1, HIDDEN_STATE_LENGTH> InputHiddenStateTensor;
        z::math::Tensor<InferencePrecision, 1, 1, HIDDEN_STATE_LENGTH> InputCellStateTensor;



        z::math::Vector<InferencePrecision, INPUT_TENSOR_LENGTH_UNIT> InputScaleVec;
        z::RingBuffer<z::math::Vector<InferencePrecision, INPUT_TENSOR_LENGTH_UNIT>> HistoryInputBuffer;

        //output tensor
        z::math::Tensor<InferencePrecision, 1, OUTPUT_TENSOR_LENGTH> OutputTensor;
        z::math::Vector<InferencePrecision, OUTPUT_TENSOR_LENGTH> OutputScaleVec;
        z::math::Tensor<InferencePrecision, 1, 1, HIDDEN_STATE_LENGTH> OutputHiddenStateTensor;
        z::math::Tensor<InferencePrecision, 1, 1, HIDDEN_STATE_LENGTH> OutputCellStateTensor;

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

