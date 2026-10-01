# 技术 1.6 验收（弱纹理环境高精度融合导航）

## 硬边界
- 紧耦合定位在 C++ 节点内运行，Python/HMI 不阻塞定位回路。
- C++ 侧通过 `ImuAdapter`/`FootForceAdapter`/`RobotSdkAdapter` 抽象接口注入，不直调厂商 SDK。
- 融合/SLAM 库未批准：`AbstractLocalizerBackend` 抽象接口，`FakeLocalizerBackend` 仅做确定性位姿输出，不虚构定位结果。
- 激光失效时不伪造激光有效状态（红线）：退化监控器如实标记 `lidar_valid=false`，降级融合维持定位。

## adapter / 订阅注入说明
`tech_1_6_node` 内部五组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **ImuAdapter / FootForceAdapter**：由上层节点启动时创建具体实例化适配器，通过 `SetImuAdapter()` / `SetFootForceAdapter()` 注入。
- **RobotSdkAdapter**：由上层 launch 或节点初始化时创建，通过 `SetRobotSdkAdapter()` 注入；运动学约束构建器读取机器人状态。
- **AbstractLocalizerBackend**：融合/SLAM 库批准后创建具体派生，通过 `SetLocalizerBackend()` 注入；未注入时使用默认 `FakeLocalizerBackend`，不虚构 SLAM 结果。
- **激光/相机**：由上层注入，未接入时标记 `valid=false`，不伪造有效状态。

## PPT 指标（阈值已落地 `localization_metrics.py`）
- 定位精度 ±**2cm**（0.02m）
- 退化条件下维持定位成功率 ≥ **90%**（`PPT_DEGRADED_MAINTAIN_RATIO`）

## 组件架构
```
MultiSensorSynchronizer → KinematicConstraintBuilder
→ TightlyCoupledLocalizer → LocalizationDegradationMonitor
→ NavigationExecutor
```
- **MultiSensorSynchronizer**：对齐激光/多目相机/1000Hz IMU/500Hz 足力时间戳
- **KinematicConstraintBuilder**：50N 足力约束 + 关节限位（复用 `RobotStateRaw`/`FootForceSample`）
- **TightlyCoupledLocalizer**：管理后端生命周期，融合视觉—激光—惯导—运动学
- **LocalizationDegradationMonitor**：识别激光失效，标记有效数据源，不伪造
- **NavigationExecutor**：跨楼层路径执行状态机，退化时暂停

## 必测场景
详见 [scenarios.md](./scenarios.md)。

## 证据模板
验收证据按本目录 `scenarios.md` 的场景逐项归档。

## 验证方式

- 纯逻辑测试（无需 ROS 2）：仓库根目录执行 `bash tools/reproduce.sh`（Linux/macOS）
  或 `powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1`（Windows），
  脚本依次运行 Python 单元测试与 C++ 冒烟测试；
- ROS 环境：`colcon build` 后 `ros2 launch inspection_bringup tech_1_6.launch.py`。
