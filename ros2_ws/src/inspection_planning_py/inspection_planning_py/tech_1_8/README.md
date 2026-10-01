# tech_1_8 动态红外测温（Python 决策侧）

## 两模块
- `thermal_metrics`：PPT 三类指标聚合
  - 范围内偏差 ≤ 0.2 ℃（`PPT_STATIC_DEVIATION_C`）
  - 动态精度 ± 0.5 ℃（`PPT_DYNAMIC_ACCURACY_C`）
  - 效率提升 ≥ 10 倍（`PPT_EFFICIENCY_GAIN`）
  - 红线：超范围样本不计入精度统计（只统计 `in_range=True` 的）
- `thermal_anomaly_decision_node`：异常高温判定（ROS 节点，离线可运行）
  - 订阅 `/thermal_measurement`（C++ 1.8 输出）
  - 仅在 `error_state == "ok"`（范围内）时判定异常高温 > 80 ℃（`ANOMALY_HIGH_TEMP_C`）
  - 发布 `/inspection_alert`（复用已有消息，不新增类型）
  - 红线：超范围时不做异常高温判定（不按范围内精度发布，也不触发异常）

## 红线
- 稳像与温度补偿在 C++ 执行，Python 只做异常高温判定（不重复稳像/补偿）。
- 超范围样本不计入精度统计，也不触发异常高温判定。
- 不虚构温度/精度；离线/在线分离；Python 用 `unittest`（非 pytest）。

## 验证方式

- 纯逻辑测试（无需 ROS 2）：仓库根目录执行 `bash tools/reproduce.sh`（Linux/macOS）
  或 `powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1`（Windows），
  脚本依次运行 Python 单元测试与 C++ 冒烟测试；
- ROS 环境：`colcon build` 后 `ros2 launch inspection_bringup tech_1_8.launch.py`。
