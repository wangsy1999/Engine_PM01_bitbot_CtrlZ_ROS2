/**
 * @file MotorSineWorker.hpp
 * @author  
 * @brief 让指定电机做正弦运动的 Worker
 */

#pragma once
#include "AbstractWorker.hpp"
#include "Schedulers/AbstractScheduler.hpp"
#include "Utils/MathTypes.hpp"
#include <cmath>
#include <atomic>
#include <nlohmann/json.hpp>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace z
{
    template<typename SchedulerType, typename MotorPrecision, size_t JointNumber>
    class MotorSineWorker : public AbstractWorker<SchedulerType>
    {
        using MotorValVec = math::Vector<MotorPrecision, JointNumber>;

    public:
        /**
         * @brief 构造函数
         *
         * JSON 配置格式示例：
         * {
         *    "MotorSine": {
         *        "AmpDeg": 30,
         *        "Freq": 0.5,
         *        "MotorId": 3
         *    }
         * }
         */
        MotorSineWorker(
            typename SchedulerType *scheduler,
            const nlohmann::json& cfg)
            : AbstractWorker<SchedulerType>(scheduler),
              enabled(false),
              t(0.0)
        {        
            A = cfg["AmpDeg"].get<MotorPrecision>() ;   // 振幅（角度）
            f = cfg["Freq"].get<MotorPrecision>();                    // 频率 Hz
/*             motor_id = cfg["MotorId"].get<int>();   */                   // 电机 id

            dt = scheduler->getSpinOnceTime();                        // 每步时间

            std::cout << "MotorSineWorker created\n";
            std::cout << "  Amp = " << A << " deg\n";
            std::cout << "  Freq = " << f << " Hz\n";
/*             std::cout << "  Motor = " << motor_id << "\n"; */
        }

        void Enable()
        {
            this->enabled = true;
            t = 0.0;
        }

        void Disable()
        {
            this->enabled = false;
        }

        /**
         * @brief TaskRun: 每个主任务周期执行一次
         */
    void TaskRun() override
    {
        std::cout << "task run\n";
        t += dt;
        MotorValVec pos;
        this->Scheduler->template GetData<"TargetMotorPosition">(pos);
        pos[2] = A * std::sin(2.0 * M_PI * f * t);
        pos[3] = A * std::sin(2.0 * M_PI * f * t);
        this->Scheduler->template SetData<"TargetMotorPosition">(pos);
    }

    private:
        std::atomic<bool> enabled;
        MotorPrecision A;      // 振幅（deg）
        MotorPrecision f;      // 频率
        MotorPrecision dt;     // 调度器周期
        MotorPrecision t;      // 时间累计
        int motor_id;          // 控制的电机 id
    };
};