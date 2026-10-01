# 技术 1.5 验收（钢格网主动抑振）

## 硬边界
- 500/1000Hz 控制环在 C++ 独立线程运行，Python/HMI 不阻塞 MPC 回路。
- C++ 侧通过 `RobotSdkAdapter`/`FootForceAdapter`/`ImuAdapter` 抽象接口注入，不直调厂商 SDK。
- MPC 求解器未批准：`AbstractMpcSolver` 抽象接口，`FakeMpcSolver` 仅做确定性阻尼修正，不虚构求解结果。

## adapter / 订阅注入说明
`tech_1_5_node` 内部四组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **FootForceAdapter / ImuAdapter**：由上层节点启动时创建具体实例化适配器，通过 `SetFootForceAdapter()` / `SetImuAdapter()` 注入；高频线程调用 `ReadSample()` 读取数据。
- **RobotSdkAdapter**：由上层 launch 或节点初始化时创建，通过 `SetRobotSdkAdapter()` 注入；`ApplyCorrection` 后通过 `SendJointCommand()` 下发。
- **AbstractMpcSolver**：训练框架/求解库批准后创建具体派生，通过 `SetMpcSolver()` 注入；未注入时 `MpcVibrationController` 不激活，不虚构修正。

## PPT 指标（阈值已落地 `grating_metrics_recorder.hpp` + `grating_metrics.py`）
- 步态异常率 ≤ **15%**（PPT：降低 85% 以上）
- 平均速度 ≥ **0.8 m/s**
- 单次巡检距离 ≥ **2 km**（2000 m）

指标由 C++ 侧经 `/grating_status`（GratingStatus）实测输出（10 Hz），Python 只做
任务级聚合与阈值判定；`valid=false` 表示尚无实测数据，不做达标判定。

## 必测场景
详见 [scenarios.md](./scenarios.md)。

## 证据模板
验收证据按本目录 `scenarios.md` 的场景逐项归档。

## 验证方式

- 纯逻辑测试（无需 ROS 2）：仓库根目录执行 `bash tools/reproduce.sh`（Linux/macOS）
  或 `powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1`（Windows），
  脚本依次运行 Python 单元测试与 C++ 冒烟测试；
- ROS 环境：`colcon build` 后 `ros2 launch inspection_bringup tech_1_5.launch.py`。
