# 四足机械人巡检系统（ROS 2 三层架构）

Copyright (c) 2026 青岛港湾职业技术学院 · 灵枢智行团队（qdgw / lszx_byc）（联系邮箱：xyt1351325@hotmail.com）。保留所有权利，授权条款见 [LICENSE](LICENSE)。

灵枢巡检机器人（LingShu Inspector）是面向工业场区巡检的四足机械狗软件系统，以 ROS 2 为通信框架，按「C++ 执行层 — Python 规划决策层 — 人机交互层」三层划分，围绕 9 项核心技术（1.1–1.9）组织代码、参数、测试与验收材料。

## 三层架构

| 层 | 语言 | 包 / 入口 | 职责 |
|---|---|---|---|
| 执行层 | C++ | `inspection_execution_cpp` | 传感器接入、行为树、安全校验、PPO / MPC / 导航控制、信号处理 |
| 规划决策层 | Python | `inspection_planning_py` | 结构化目标生成、任务调度、重规划、PPO 训练、诊断决策 |
| 人机交互层 | JavaScript | ROS WebSocket 网关（`hmi_bridge.launch.py`） | 指令输入、状态 / 任务 / 地图 / 热像 / 声纹展示 |

1000 Hz IMU、500 Hz 足端力与运动控制、融合估计、点云与音频前处理全部下沉到 C++；Python 只发布结构化目标、任务计划、重规划结果与诊断结论；交互层不直接控制关节、不绕过行为树、不修改安全约束。

## 九项核心技术

| 编号 | 技术 | 主要落点 |
|---|---|---|
| 1.1 | 大模型—行为树—PPO 双环协同 | Python 结构化目标与在线决策 + C++ BehaviorTree.CPP v4 执行 |
| 1.2 | 风险与能耗约束的动态任务抢占与恢复 | 任务价值评分、能耗约束、抢占 / 恢复调度、低电量返航 |
| 1.3 | 面向多场景的机器人运动智能迁移 | 地形参数化与随机化、域随机化、PPO 训练与策略导出 + C++ 策略运行时 |
| 1.4 | 极窄通道智能感知与通行 | 点云分割、视觉纹理验证、走廊融合、窄道执行 |
| 1.5 | 钢格网地面主动抑振 | 足接触与振动估计、MPC 抑振，独立高频线程 |
| 1.6 | 弱纹理环境高精度融合导航 | 多源时间同步、50 N 运动学约束、紧耦合定位、退化监控 |
| 1.7 | 语义地图构建与定位 | 点云栅格赋权、BIM–SLAM 关联、报警位置匹配 |
| 1.8 | 抗振抑扰的动态红外精准测温 | 时间同步与稳像、辐射率 / 角度距离补偿、范围校验 |
| 1.9 | 强噪声环境定向声纹早期诊断 | 麦克风同步、定向波束形成、空间滤波、SNR 估计与故障分类 |

九项技术使用同一套工程约定：C++ 组件 + Python 决策模块 + launch / CMakeLists / setup.py 注册 + 单元测试 + 验收场景清单。

## 仓库结构

```text
.
├─ docs/                        接口、坐标系、验收指标、数据参数清单
├─ ros2_ws/src/
│  ├─ inspection_interfaces/    共享 msg / srv / action
│  ├─ inspection_execution_cpp/ C++ 执行层
│  ├─ inspection_planning_py/   Python 规划决策层
│  ├─ inspection_bringup/       启动与参数装配
│  └─ inspection_tests/         单元 / 集成 / 性能 / 验收
└─ tools/                       构建、测试与接口检查脚本
```

## 运行环境

| 项 | 版本 / 说明 |
|---|---|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill（`source /opt/ros/humble/setup.bash`） |
| Python | 3.10（Humble 自带） |
| C++ 标准 | C++17，编译器 clang++ 14+ 或 g++ 11+ |
| CMake | ≥ 3.8 |
| 构建工具 | colcon-common-extensions、python3-rosdep |
| 行为树库 | BehaviorTree.CPP v4（Humble 官方源仅含 v3，需源码安装） |
| 大模型后端 | DeepSeek（可选，缺省走 stub） |

ROS 2 依赖一次性安装：

```bash
sudo apt update && sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-rclcpp ros-humble-rclpy \
  ros-humble-std-msgs ros-humble-geometry-msgs \
  ros-humble-sensor-msgs ros-humble-nav-msgs \
  ros-humble-tf2 ros-humble-tf2-ros \
  ros-humble-action-msgs ros-humble-behaviortree-cpp-v3
```

BehaviorTree.CPP v4 源码安装（1.1 执行层依赖 v4 头文件前缀 `behaviortree_cpp/`）：

```bash
cd ~ && git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git bt_cpp_v4
cd bt_cpp_v4 && git checkout 4.6.2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DBTCPP_EXAMPLES=OFF -DBTCPP_UNIT_TESTS=OFF \
  -DBTCPP_GROOT_INTERFACE=OFF -DBTCPP_SQLITE_LOGGING=OFF
make -j$(nproc) && sudo make install && sudo ldconfig
```

## 构建与运行

```bash
# 1) source ROS 2 环境后构建
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install
source install/setup.bash

# 2) 启动三层连通骨架
ros2 launch inspection_bringup system.launch.py

# 3) 运行 ROS 环境测试
colcon test --event-handlers console_direct+
```

纯逻辑测试（无需 ROS 2）：`bash tools/reproduce.sh`，Windows 用 `powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1`。

按技术单独启动：`ros2 launch inspection_bringup tech_1_1.launch.py`（`tech_1_1` 至 `tech_1_9`）。

## 配置与参数

- 启动参数：`ros2_ws/src/inspection_bringup/config/`（QoS、坐标系、设备、验收阈值）；
- 规划参数：`ros2_ws/src/inspection_planning_py/config/`（任务评分、规划、模型路径、安全电量）；
- 全部阈值与内置常量的清单见 [数据参数清单](docs/数据参数清单.txt)，代码不另设验收口径。

## 大模型后端（DeepSeek）

结构化目标生成支持 DeepSeek 后端，密钥只通过环境变量注入，不写入仓库：

- `DEEPSEEK_API_KEY`：DeepSeek API Key；
- 节点参数：`llm_backend`（默认 `deepseek`，可切 `stub`）、`llm_model`（默认 `deepseek-chat`）、`llm_timeout_sec`。

## 行为树执行（BehaviorTree.CPP）

1.1 执行层使用 BehaviorTree.CPP v4（头文件前缀 `behaviortree_cpp/`，目标 `behaviortree_cpp::behaviortree_cpp`），构建前需在 ROS 2 环境安装该库；若使用 v3，请把 include 前缀改为 `behaviortree_cpp_v3/`。树文件位于 `inspection_execution_cpp/behavior_trees/`。

## 测试状态

| 项 | 范围 | 结果 |
|---|---|---|
| Python 单元测试 | 20 个测试文件、158 项用例（`unittest`，含节点级与工程一致性检查） | 本机全部通过 |
| C++ 纯逻辑 smoke test | `smoke_test_core.cpp` + 38 个组件源文件，`-std=c++17 -Wall -Wextra -Wpedantic` | 零告警，断言全部通过 |
| 集成 / 性能测试 | 需 ROS 2 环境，含占位桩 | 本机未运行 |

PPT 中的定量指标（成功率、精度、时延、效率等）需在实机或 ROS 环境采集真实数据后核验，仓库中不预置任何未实测的数值。

## 复现步骤

以下步骤对应「代码结构说明 + 运行环境说明 + 主要结果复现」三项要求，按从轻到重排列；前两步可在无 ROS 2 的开发机上完成，第三步起需 Ubuntu + ROS 2 Humble。

### 1. 代码结构与关键算法定位

- 三层包与九项技术落点见上文「三层架构」「九项核心技术」两节；
- 每项技术的 C++ 组件、Python 模块、launch、单元测试、验收场景均在对应 `tech_1_x/` 目录，命名一致；
- 阈值与内置常量集中在 [docs/数据参数清单.txt](docs/数据参数清单.txt)，代码不另设验收口径。

### 2. 纯逻辑单元测试（无需 ROS 2）

一键复现（推荐，脚本会依次跑完下面两步并给出通过结论）：

```bash
bash tools/reproduce.sh                                         # Linux / macOS
powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1     # Windows
```

以下两条命令与脚本内容一致，便于逐步核对：

Python 单元测试（`unittest`，Windows / Linux 均可）：

```bash
cd ros2_ws/src
PYTHONPATH=inspection_planning_py python -m unittest discover \
  -s inspection_tests/unit_python/tech_1_1_to_1_9 -p "test_*.py" -v
# 实测：158 项用例全部通过（含 1.2 安全电量策略 21 例、节点级与一致性检查 24 例）
```

C++ 纯逻辑冒烟测试（`clang++` 或 `g++`，脱离 ROS 编译）：

```bash
cd ros2_ws/src
g++ -std=c++17 -Wall -Wextra -Wpedantic \
  inspection_tests/unit_cpp/tech_1_1_to_1_9/smoke_test_core.cpp \
  $(find inspection_execution_cpp/src -name '*.cpp' \
    | grep -vE '_node|bt_task_executor|task_lifecycle_executor') \
  -I inspection_execution_cpp/include \
  -o /tmp/smoke_test_core && /tmp/smoke_test_core
# 实测：零告警，全部断言通过（覆盖 1.1–1.9 纯逻辑组件共 38 个源文件；
#       依赖 ROS / BehaviorTree.CPP 的执行器组件需在 colcon build 后用 colcon test 验证）
```

### 3. ROS 2 集成与三层连通（需 Ubuntu + ROS 2 Humble）

以下命令已在 Ubuntu 22.04.5 LTS + ROS 2 Humble 环境实测通过。

```bash
# 1) 安装运行环境（见上「运行环境」节）后构建
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install
# 实测：5 个包全部编译通过（inspection_interfaces / inspection_execution_cpp /
#       inspection_planning_py / inspection_tests / inspection_bringup）

source install/setup.bash

# 2) 启动三层连通骨架
ros2 launch inspection_bringup system.launch.py
# 实测：10 个节点全部启动，10 个 topic 正常发布
#   节点：cpp_heartbeat_node / tech_1_1_node / tech_1_2_node / py_heartbeat_node /
#        llm_online_decision_node / goal_replanner_node / candidate_task_pool_node /
#        energy_constraint_node / preemption_scheduler_node / rosbridge_websocket
#   topic：/robot_state /operator_instruction /structured_goal /task /task_decision /
#          /energy_constraint /inspection_alert ...

# 3) 按技术单独启动并复现对应场景
ros2 launch inspection_bringup tech_1_1.launch.py   # 1.1 至 1.9 各有独立 launch

# 4) 1.2 安全电量返航集成测试
python3 -m pytest src/inspection_tests/integration/test_low_battery_return.py -v
# 实测：4/4 通过（低于 30% 返航、恰好 30% 返航、动态阈值抬高返航、电量恢复后正常执行）
```

### 4. 九项技术验收材料

每项技术的必测场景与验收说明位于 `ros2_ws/src/inspection_tests/acceptance/tech_1_1` 至 `tech_1_9`；验收总览见 [acceptance/README.md](ros2_ws/src/inspection_tests/acceptance/README.md)。

## 文档

- [接口说明](docs/interfaces.md)
- [坐标系说明](docs/coordinate_frames.md)
- [验收指标](docs/acceptance_metrics.md)
- [数据参数清单](docs/数据参数清单.txt)、[参数数据表](docs/参数数据表.txt)
- [使用须知](docs/须知.txt)

## 约定

- 不提交模型权重、数据集与密钥，`models/` 只保留目录结构与说明；
- 缺少实测数据时不虚构指标，失效数据源如实标记；
- 合作方式与说明见 [docs/须知.txt](docs/须知.txt)。
