# 技术 1.2 必测场景清单

## S1：C++ TaskProgressStore 进度存储（smoke）
- 输入：`TaskProgress{0.5, "running"}` 存为 "task-a"
- 期望：
  - `Has("task-a")==true`，`Load("task-a", &loaded)` 成功且 `loaded.progress==0.5`；
  - `Clear("task-a")` 后 `Has()==false`。
- 证据：`smoke_test_core.cpp` 断言通过截图。

## S2：C++ TaskResumeExecutor 恢复执行器（smoke）
- 输入：store 已存 "task-a" 进度
- 期望：
  - `CanResume("task-a")==true`；
  - `LoadResumePoint("task-a", &rp)` 成功且 `rp.progress==0.5`。
- 证据：`smoke_test_core.cpp` 断言通过截图。

## S3：C++ PreemptionLatencyMonitor 抢占时延（smoke）
- 输入：`MarkArrival(t0)` + `MarkSwitchComplete(t0 + 300ms)`
- 期望：
  - `Count()==1`；
  - `LastLatencyMs() ∈ (250, 350)`（时延如实记录，不虚构）。
- 证据：`smoke_test_core.cpp` 断言通过截图。

## S4：Python 抢占调度五种决策（unittest）
- 输入：多组任务（benefit/risk/urgency）+ 电量/返航距离 + 可恢复任务
- 期望：
  - 低电量（battery=2%）→ `CHARGE`；
  - 无当前任务 + 多任务 → `EXECUTE` 选最高价值；
  - 当前任务低价值 + 高价值任务 → `PREEMPT`；
  - 无任务 → `WAIT`；
  - 有可恢复任务 → `RESUME`。
- 证据：`test_preemption_scheduler.py` 全通过截图（5 个用例）。

## S5：Python SchedulerMetrics 多任务成功率与最大抢占（unittest）
- 输入：`SchedulerMetrics` 记录 (True)/(False)/(True) + 抢占 300/100ms
- 期望：
  - `task_success_rate() == 2/3`；
  - `max_preemption_ms() == 300.0`。
- 证据：`test_decision_metrics.py`（含 `TestSchedulerMetrics`）全通过截图。

## S6：ROS launch 装配 + Topic 命名
- 输入：`ros2 launch inspection_bringup tech_1_2.launch.py`
- 期望：
  - 节点名取自公共 `node_names.hpp`（`kTech12NodeName`，无散写字符串）；
  - 订阅 `/task /energy_constraint`，发布 `/task_decision`（公共 `topic_names` 集中定义）。
- 证据：`ros2 node list`、`ros2 topic list` 截图；CMakeLists `add_executable` 片段。

## S7：抢占集成测试 EXECUTE→PREEMPT（需 ROS 2 环境）
- 输入：`test_task_preemption.py` 发布低价值任务后注入高价值任务
- 期望：先观测到 `EXECUTE`（current_task_id="low"），再观测到 `PREEMPT`（current_task_id="high"）。
- 证据：`pytest test_task_preemption.py` 通过截图（需 ROS 2 环境运行）。

## S8：抢占响应 ≤500ms 性能（需 ROS 2 环境）
- 输入：`test_500ms_switch.py` 低价值任务运行 0.3s 后注入高价值任务
- 期望：观测到 `PREEMPT` 决策，注入到决策时延 `≤ 500ms`（PPT 阈值）。
- 证据：`pytest test_500ms_switch.py` 通过截图（需 ROS 2 环境运行）。

## S9：多任务成功率 92.3% + 抢占≤500ms 实机闭环（最终指标）
- 输入：PPT 场景（多任务并发、突发高优先级、原任务恢复、低电量）× 多轮
- 期望：按 S5 统计多任务成功率 `≥ 92.3%`；抢占响应 `≤ 500ms` 全通过；附调度日志、`TaskDecision` 反馈、`TaskProgressStore` 恢复记录。
- 证据：现场记录与统计表按本场景归档。
