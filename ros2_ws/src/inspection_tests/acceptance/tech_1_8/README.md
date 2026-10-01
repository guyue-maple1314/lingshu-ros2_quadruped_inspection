# 技术 1.8 验收（抗振抑扰的动态红外精准测温）

## 硬边界
- 稳像与温度补偿在 C++ 执行，Python 决策侧只做异常高温判定（不重复稳像/补偿，红线）。
- 超出 PPT 范围（角度 ±60°、距离 1–5 m、修正系数 0.95–1.05、辐射率 (0,1]）时标记 `xxx_out_of_range`，`in_range=false`，不按范围内精度发布（红线）。
- IMU 无效（适配器未注入 / `ReadSample` 失败）时 `TrySync` 返回 `nullopt`，不发布测温结果（不虚构温度/精度，红线）。
- 辐射率非物理值时 `EmissivityCompensator` 标 `invalid`；`AngleDistanceCompensator` 中 `cos(angle)` 下限 0.01 防发散。

## adapter / 注入说明
`tech_1_8_node` 内部五组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **ImuAdapter**：通过 `SetImuAdapter()` 注入；未注入或 `ReadSample` 失败时不发布（与 tech_1_3_node / tech_1_5_node / tech_1_6_node 同理）。
- **ThermalFrame**：通过 `SetThermalFrame()` 注入；未注入时 `valid=false`，不做测温计算。
- **ThermalMeasurement**：复用已有消息接口，由节点发布，不新增消息类型。

## PPT 指标（阈值已落地 `thermal_metrics.py`）
- 范围内偏差 ≤ **0.2 ℃**（`PPT_STATIC_DEVIATION_C`）
- 动态精度 ± **0.5 ℃**（`PPT_DYNAMIC_ACCURACY_C`）
- 效率提升 ≥ **10 倍**（`PPT_EFFICIENCY_GAIN`）

## 组件架构
```
ThermalVisualImuSynchronizer → ThermalImageStabilizer
→ EmissivityCompensator → AngleDistanceCompensator
→ ThermalRangeValidator（发布 ThermalMeasurement）
```
- **ThermalVisualImuSynchronizer**：30fps 红外 + 1000Hz IMU 滑窗时间同步，IMU 无效返回 `nullopt`
- **ThermalImageStabilizer**：IMU 角速度→位移补偿，IMU/红外无效时 `valid=false`
- **EmissivityCompensator**：`compensated = raw / ε^0.25`，非物理辐射率标 `invalid`
- **AngleDistanceCompensator**：`compensated = temp / cos(angle) * correction_factor`，`cos` 下限 0.01
- **ThermalRangeValidator**：PPT 范围校验（±60° / 1–5m / 0.95–1.05 / ε∈(0,1]），超范围标 `xxx_out_of_range`

## 必测场景
详见 [scenarios.md](./scenarios.md)。

## 证据模板
验收证据按本目录 `scenarios.md` 的场景逐项归档。

## 验证方式

- 纯逻辑测试（无需 ROS 2）：仓库根目录执行 `bash tools/reproduce.sh`（Linux/macOS）
  或 `powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1`（Windows），
  脚本依次运行 Python 单元测试与 C++ 冒烟测试；
- ROS 环境：`colcon build` 后 `ros2 launch inspection_bringup tech_1_8.launch.py`。
