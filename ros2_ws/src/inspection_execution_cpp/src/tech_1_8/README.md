# tech_1_8 抗振抑扰的动态红外精准测温（C++）

## 五组件
- `thermal_visual_imu_synchronizer`：30fps 红外 + 1000Hz IMU 滑窗时间同步（容差可配），IMU 无效返回 `nullopt`（不虚构）
- `thermal_image_stabilizer`：IMU 角速度→位移补偿（曝光时间 × 焦距因子），IMU/红外无效 `valid=false`
- `emissivity_compensator`：`compensated = raw / ε^0.25`，非物理辐射率标 `invalid`
- `angle_distance_compensator`：`compensated = temp / cos(angle) * correction_factor`，`cos` 下限 0.01 防发散
- `thermal_range_validator`：PPT 范围校验（±60° / 1–5m / 0.95–1.05 / ε∈(0,1]），超范围标 `xxx_out_of_range`，`in_range=false`

## adapter / 注入说明
`tech_1_8_node` 内部五组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **ImuAdapter**：通过 `SetImuAdapter()` 注入；未注入或 `ReadSample` 失败时不发布（与 tech_1_3_node / tech_1_5_node / tech_1_6_node 同理）。
- **ThermalFrame**：通过 `SetThermalFrame()` 注入；未注入时 `valid=false`，不做测温计算。
- **ThermalMeasurement**：复用已有消息接口，由节点发布，不新增消息类型。

## 红线
- IMU 无效时不发布测温结果（不虚构温度/精度）。
- 超出 PPT 范围（±60°、1–5m、0.95–1.05、ε∈(0,1]）时标记 `xxx_out_of_range`，`in_range=false`，不按范围内精度发布。
- 稳像与温度补偿在 C++ 执行，Python 只做异常高温判定（不重复稳像/补偿）。
- 辐射率非物理值标 `invalid`；`cos(angle)` 下限 0.01 防发散。

## 验证方式

- 纯逻辑测试（无需 ROS 2）：仓库根目录执行 `bash tools/reproduce.sh`（Linux/macOS）
  或 `powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1`（Windows），
  脚本依次运行 Python 单元测试与 C++ 冒烟测试；
- ROS 环境：`colcon build` 后 `ros2 launch inspection_bringup tech_1_8.launch.py`。
