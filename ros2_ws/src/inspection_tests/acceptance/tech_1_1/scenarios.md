# 技术 1.1 必测场景清单

## S1：C++ GoalSafetyValidator 目标安全校验（smoke）
- 输入：`GoalSafetyInput{goal_id="g-1", task_type="inspection", valid_until_sec=100, now_sec=0, robot_ready=true}`
- 期望：
  - `Validate(input).allowed == true`；
  - `task_type="unknown"` 或 `robot_ready=false` 时 `allowed == false`（不放过非法/未就绪目标）。
- 证据：`smoke_test_core.cpp` 断言通过截图。

## S2：C++ AbnormalSwitchMonitor 异常切换时延（smoke）
- 输入：`MarkArrival(t0)` + `MarkSwitchComplete(t0 + 100ms)`
- 期望：`MaxLatencyMs() > 50.0`（时延被如实记录，不虚构）。
- 证据：`smoke_test_core.cpp` 断言通过截图。

## S3：C++ PpoPolicyRuntime 抽象接口不虚构推理（smoke）
- 输入：`PpoPolicyRuntime` 已加载 metadata 但未注入 `inference_hook`
- 期望：`Load()==true` 但 `IsLoaded()==false`，`Infer(...)==false`，act 不被写入假数据。
- 红线：任何情况下不伪造策略输出（与 1.3 `PpoPolicyLoaderImpl` 一致）。
- 证据：`smoke_test_core.cpp` 断言通过截图（含 `tech_1_1 components`）。

## S4：Python 结构化目标 Schema 校验（unittest）
- 输入：合法 goal（goal_id/task_type/target_pose/constraints/valid_until_sec 齐全）
- 期望：
  - `validate_goal_schema(goal)` 返回 `(True, "")`；
  - 删除 `goal_id` 或 `task_type="unknown"` 时返回 `(False, ...)`；
  - `to_goal_draft(goal).goal_id == "g-1"`。
- 证据：`test_goal_schema.py` 全通过截图。

## S5：Python LLM 抽象客户端（unittest）
- 输入：`build_goal_prompt({instruction, alerts, robot_state})` + 多种 LLM 返回（纯 JSON / ```json fenced``` / 非法）
- 期望：
  - `build_goal_prompt` system 提示含 "JSON"，user 为合法 JSON 串；
  - `parse_goal_response` 解析纯 JSON 与 fenced JSON 均成功，非法返回 `None`；
  - `StubLlmClient` 不直连真实 API（离线/在线分离红线）。
- 证据：`test_llm_client.py` 全通过截图。

## S6：Python 任务价值评分 + 能耗约束（unittest）
- 输入：多组任务（benefit/risk/urgency）与电量/返航距离
- 期望：`task_value_evaluator` 价值排序正确；`energy_constraint_evaluator` 低电量触发返航/充电约束。
- 证据：`test_task_value_evaluator.py` + `test_energy_constraint_evaluator.py` 全通过截图。

## S7：Python 决策上下文 + 成功率/p95 指标（unittest）
- 输入：`DecisionMetrics` 记录 (True,100)/(False,200)/(True,150)
- 期望：
  - `success_rate() == 2/3`；
  - `p95_ms() == 195.0`（线性插值，sorted=[100,150,200] → 0.1×150+0.9×200）；
  - `decision_context_builder` 上下文字段齐全。
- 证据：`test_decision_metrics.py` + `test_decision_context_builder.py` 全通过截图。

## S8：ROS launch 装配 + Topic 命名
- 输入：`ros2 launch inspection_bringup tech_1_1.launch.py`
- 期望：
  - 节点名取自公共 `node_names.hpp`（无散写字符串）；
  - 订阅 `/operator_instruction`、发布 `/robot_state` 与决策相关 topic（公共 `topic_names` 集中定义）。
- 证据：`ros2 node list`、`ros2 topic list` 截图；CMakeLists `add_executable` 片段。

## S9：异常切换 ≤500ms 性能（需 ROS 2 环境）
- 输入：`test_500ms_switch.py` 发布低价值任务后注入高价值任务
- 期望：观测到 `PREEMPT` 决策，且从注入到决策时延 `≤ 500ms`（PPT 阈值）。
- 证据：`pytest test_500ms_switch.py` 通过截图（需 ROS 2 环境运行）。

## S10：结构化指令成功率 ≥95% 实机闭环（最终指标）
- 输入：PPT 场景（状态变化、设备告警、语音/文字指令）× 多轮
- 期望：按 S4-S7 统计结构化指令生成成功率 `≥ 95%`；附指令样本集、时间戳日志、`DecisionMetrics` 汇总。
- 证据：现场记录与统计表按本场景归档。
