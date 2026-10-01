# 技术 1.3 验收（多场景运动智能迁移）

## 硬边界
- 离线训练侧（Python）：`inspection_planning_py/tech_1_3/*`，纯模块/脚本，不连接实机关节执行接口。
- 在线运行时侧（C++）：`inspection_execution_cpp/tech_1_3/*`，通过 `RobotSdkAdapter` 抽象下发，不直调厂商 SDK。
- 训练框架未批准：`AbstractTrainer` 接口化，无虚构推理结果；`PpoPolicyLoaderImpl::IsLoaded()` 无权重钩子时恒为 `false`。

## adapter / 订阅注入说明
`tech_1_3_node` 内部的四个组件（`PpoPolicyLoader`/`ObservationBuilder`/`PolicyOutputValidator`/`LocomotionCommandAdapter`）采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 `/imu`、`/foot_force`，也不实例化真实 `RobotSdkAdapter`。这是刻意为之——不虚构实机模型。真实装配方式：
- **IMU / 足力数据**：由 `tech_1_3_node` 的 ROS2 订阅回调（`/imu`、`/foot_force`）接收消息后，调用 `ObservationBuilder::SetImu(...)` / `SetFootForce(...)` 注入。
- **RobotSdkAdapter 实例**：由上层 launch 或节点启动时创建具体厂商适配器（如 `UnitreeSdkAdapter`），通过 `LocomotionCommandAdapter` 构造函数注入；当前仅定义抽象接口，真实 adapter 由后续实机联调阶段在节点初始化时实例化。
- **策略权重**：通过 launch 参数 `policy_dir` 传入导出目录路径，`PpoPolicyLoaderImpl::Load(dir)` 加载 metadata.json；权重推理钩子待训练框架批准后注入。

## PPT 指标（阈值已落地 `transfer_metrics.py`）
- 复杂地形通行率 ≥ **90%**（PPT：复杂地形通行率 >90%）
- 实机迁移成功率 ≥ **96.8%**（PPT：96.8%）
- Sim-to-Real 域随机化覆盖：地形/质量/摩擦/惯量/IMU 偏置 ≥ 5 维度
- 策略输出校验拦截率：100%（超范围 / 超维 / 超时的动作绝不透传到 RobotSdkAdapter）

## 必测场景
详见 [scenarios.md](./scenarios.md)。

## 证据模板
验收证据按本目录 `scenarios.md` 的场景逐项归档。

## 离线验证
- Python 21/21 单测：
  ```
  set PYTHONPATH=...\ros2_ws\src\inspection_planning_py
  cd ros2_ws\src\inspection_tests\unit_python
  python -m unittest tech_1_1_to_1_9.test_terrain_1_3 tech_1_1_to_1_9.test_randomizers_1_3 tech_1_1_to_1_9.test_trainer_export_metrics_1_3 -v
  ```
- Python 离线训练模块运行方式（不注册 console_scripts，非 ROS 节点）：
  ```
  set PYTHONPATH=...\ros2_ws\src\inspection_planning_py
  python -m inspection_planning_py.tech_1_3.ppo_training_pipeline    # 跑 FakeTrainer 示例
  python -m inspection_planning_py.tech_1_3.policy_exporter            # 导出 metadata
  python -m inspection_planning_py.tech_1_3.transfer_metrics          # 统计迁移指标
  ```
- C++ 纯逻辑 smoke test（clang++ 脱离 ROS，已通过）：
  ```
  clang++ -std=c++17 -I<include> smoke_test_core.cpp <common/1.1/1.2/1.3 实现> -o smoke_test.exe
  smoke_test.exe  # 应打印 "core logic smoke test passed (incl. tech_1_3 components)"
  ```
- 实机 / ROS 真实编译：需在有 ROS 2 + colcon 的机器上执行。
  ```
  colcon build --packages-up-to inspection_execution_cpp inspection_planning_py
  ros2 launch inspection_bringup tech_1_3.launch.py policy_dir:=<policy_export_dir>
  ```
