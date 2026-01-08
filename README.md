# RL Controller for EngineAI Robots


## 📖 简介 (Introduction)

本项目旨在构建一套基于深度强化学习（RL）的机器人控制框架，专注于 **EngineAI（恩井科技）** 系列机器人（如 SE01 等）的仿真与真机部署。

项目基于 ROS2 架构，实现了从仿真训练到 Sim2Real 的完整流程。目前的重点在于打通仿真与真机的控制接口，并实现鲁棒的运动控制。

This repository contains the RL control framework for EngineAI robots, bridging simulation and real-world deployment via ROS2.

## 📅 项目进展 (Roadmap & Status)

截至目前，项目主要完成了 ROS2 环境下的仿真测试。后续重点在于真机部署与推理模块的完善。

- [x] **仿真测试**
    - [x] 验证基础关节控制与状态反馈。
    - [x] 验证通信链路稳定性。
- [ ] **模型推理 (Inference)**
    - [ ] 实现 RL Policy 在 C++/Python 节点中的加载 (ONNX/LibTorch)。
    - [ ] 仿真环境下的闭环控制测试。
- [ ] **真机部署 (Real Robot)**
    - [ ] 适配 EngineAI 真机 SDK / 硬件通讯接口。
    - [ ] 解决 Sim2Real 的通信延迟与状态估计问题。
    - [ ] 挂架安全测试与实机验证。

## 🛠️ 依赖 (Dependencies)

* **Operating System**: Ubuntu 22.04 (Recommended) / 20.04
* **Middleware**: ROS2 Humble / Iron
* **Simulation Env**: [engineai_ros2_workspace (branch: community)](https://github.com/engineai-robotics/engineai_ros2_workspace/tree/community)
* **Hardware**: EngineAI Robot (e.g., SE01)
* **Languages**: C++ 17, Python 3.10+

## 🚀 使用指南 (Usage)

### 1. 编译 (Build)

```bash
# 1. 创建并进入工作空间
mkdir -p engine_ws/src && cd engine_ws/src

# 2. 克隆仿真环境 (使用 community 分支)
git clone https://github.com/engineai-robotics/engineai_ros2_workspace.git

# 3. 安装仿真环境（参考https://github.com/engineai-robotics/engineai_ros2_workspace）

# 4. 克隆本项目
# git clone [https://github.com/YourUsername/YourRepo.git](https://github.com/YourUsername/YourRepo.git) .

# 5. 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
### 2. 运行仿真 (Simulation)

source install/setup.bash

### 3. 启动仿真环境与控制器 (请根据实际 Launch 文件名修改)
ros2 launch engine_rl_controller simulation.launch.py
📂 文件结构 (Structure)

```text
.
├── engine_rl_controller/    # RL 推理核心节点 (Policy Inference)
├── engine_hardware/         # 硬件抽象层 (Hardware Interface for Sim/Real)
├── engine_msgs/             # 自定义 ROS2 消息与服务
├── scripts/                 # 训练脚本与辅助工具
└── README.md
```

## 🔗 参考项目 (References)
本项目在开发过程中深入参考了以下优秀的开源项目，特此致谢：

* **[CtrlZ](https://github.com/ZzzzzzS/CtrlZ)**: 提供了核心的控制架构思路与工程实现参考。
* **[bitbot-unitree](https://github.com/ZzzzzzS/bitbot-unitree)**: 提供了 Sim2Sim 到 Sim2Real 的适配流程参考。
* **[bitbot_booster](https://github.com/Dknt0/bitbot_booster)**: 提供了高性能优化方案与工具链支持。

## ⚠️ 免责声明 (Disclaimer)

**安全第一**：真机调试具有一定的物理风险。在进行 Sim2Real 部署时，请务必遵守以下原则：

1. 始终在 **悬挂状态 (Gantry)** 或有安全保护绳的情况下进行初步测试。
2. 确保 **急停按钮 (E-Stop)** 随时可触达。
3. 开发者不对因使用本代码导致的硬件损坏或人身伤害承担责任。
