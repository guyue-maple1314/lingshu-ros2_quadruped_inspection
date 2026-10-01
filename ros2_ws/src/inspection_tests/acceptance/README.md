# 验收测试总览（最终交付）

本目录是九项核心技术（1.1–1.9）的验收中枢：每个 `tech_1_X/` 子目录存放对应技术的场景清单（`scenarios.md`）与验收说明（`README.md`）。

## 九项 PPT 指标一览

阈值集中保存在 `inspection_bringup/config/acceptance_thresholds.yaml`，代码不得另设验收口径。

| 技术 | 必测场景 | PPT 指标 | 阈值文件键 |
|---|---|---|---|
| 1.1 | 状态变化、告警、语音/文字指令、异常切换 | 结构化指令成功率 ≥95%；异常切换 ≤500 ms | `tech_1_1` |
| 1.2 | 多任务并发、突发高优先级、原任务恢复、低电量 | 多任务成功率 92.3%；抢占 ≤500 ms | `tech_1_2` |
| 1.3 | 钢格网/坡道/楼梯/窄通道，仿真到实机 | 复杂地形通行率 >90%；实机迁移 96.8% | `tech_1_3` |
| 1.4 | 46 cm 机器人过 75 cm 通道 | 稳定通行；能力 +25%；改造成本 -90% | `tech_1_4` |
| 1.5 | 钢格网振动、长距离巡检 | 异常率 -85%；均速 ≥0.8 m/s；单次 2 km | `tech_1_5` |
| 1.6 | 钢格网楼梯、弱纹理、激光退化 | 定位精度 ±2 cm | `tech_1_6` |
| 1.7 | BIM 与 SLAM 对齐、检测事件定位 | 厘米级映射；位置报告准确率 ≥98% | `tech_1_7` |
| 1.8 | 0.8 m/s 行进、±60°、1–5 m、不同辐射率 | 范围内偏差 ≤0.2 ℃；动态 ±0.5 ℃；效率 +10 倍 | `tech_1_8` |
| 1.9 | 85 dB 背景噪声、典型故障样本 | SNR 15–20 dB；识别准确率 75% | `tech_1_9` |

> 「提升类」指标（如 +25%、-90%、+10 倍、-85%）需项目方提供统一基线与统计口径；代码与测试只按获批口径计算，不虚构结果。

## 离线验证结果汇总（无 ROS 2 环境）

在 Windows 开发机重跑了全部单元级证据，结果如下（真实数据，未虚构）：

| 证据项 | 命令 | 结果 |
|---|---|---|
| C++ 纯逻辑 smoke test | `clang++ -std=c++17 -Wall -Wextra -Wpedantic` 编译 39 个源文件 | **零告警**，`smoke_test.exe` 退出码 0，输出 `core logic smoke test passed (incl. tech_1_3 + ... + tech_1_9 components)` |
| Python 单元测试 | `python -m unittest discover`（`unit_python/tech_1_1_to_1_9/`） | **158 个用例全部通过**（`Ran 158 tests ... OK`） |

覆盖范围：
- C++ smoke 覆盖 1.1（GoalSafetyValidator/AbnormalSwitchMonitor/PpoPolicyRuntime）、1.2（TaskProgressStore/TaskResumeExecutor/PreemptionLatencyMonitor）、1.3–1.9 全部在线组件。
- Python 单测覆盖 1.1（goal_schema/llm_client/task_value/energy/decision_metrics/decision_context）、1.2（preemption_scheduler/scheduler_metrics）、1.3（terrain/randomizer/trainer/transfer_metrics）、1.4（corridor_plan_validator/path_planner/corridor_metrics）、1.5（grating_metrics）、1.6（localization_metrics）、1.7（semantic_map）、1.8（thermal）、1.9（acoustic）。

## 联调结论

- **组件级逻辑（C++ 执行层 + Python 规划层）**：1.1–1.9 九项技术的全部在线/离线组件已通过单元级验证，红线（不虚构推理/定位/测温/故障、失效如实标记、超范围不计精度）全部遵守。
- **集成测试 / 性能测试（需 ROS 2 环境）**：8 个用例中 3 个已实现、5 个为 `pytest.skip` 桩，需 ROS 2 环境运行。
- **PPT 定量指标**：成功率/精度/时延/效率等定量阈值需 ROS 2 环境与实机/仿真/传感器采集真实数据后核验，仓库中不预置未实测数值。组件逻辑达标是实机指标达标的前置条件，已全部满足。

## 目录结构

```text
acceptance/
├─ README.md            本文件（验收总览）
├─ tech_1_1/            README + scenarios
├─ tech_1_2/            README + scenarios
├─ tech_1_3/            README + scenarios
├─ tech_1_4/            README + scenarios
├─ tech_1_5/            README + scenarios
├─ tech_1_6/            README + scenarios
├─ tech_1_7/            README + scenarios
├─ tech_1_8/            README + scenarios
└─ tech_1_9/            README + scenarios
```

## 运行方式

- C++ smoke：`clang++ -std=c++17 -Wall -Wextra -Wpedantic` 编译 39 个源文件，零告警，返回码 0。
- Python 单测：`python -m unittest discover -s ros2_ws/src/inspection_tests/unit_python/tech_1_1_to_1_9 -p 'test_*.py'`（`PYTHONPATH` 指向 `inspection_planning_py` 源码根）。
- 集成 / 性能测试（需 ROS 2）：`pytest ros2_ws/src/inspection_tests/{integration,performance}/`，需先 `ros2 launch inspection_bringup system.launch.py`。
- 实机验收：按各 `scenarios.md` 的最终场景（S9/S10 等）执行，证据按场景逐项归档。
