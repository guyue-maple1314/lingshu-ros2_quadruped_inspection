# tech_1_7 高精度时空融合语义地图与定位（C++）

## 四组件
- `pointcloud_transformer`：点云坐标转换（位姿 lost 时不输出，避免虚假点云）
- `weighted_grid_builder`：栅格赋权（无数据源时 `valid=false`）
- `position_matcher`：BIM 先验与实时 SLAM 关联（半径 30cm），位置结果带 `confidence` + `coordinate_source`（`bim_id`/`slam_id`/`unlocalized`）
- `alarm_location_publisher`：发布 `SemanticAlarm`（复用已有消息，不新增类型）

## adapter / 注入说明
`tech_1_7_node` 内部四组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **AbstractBimLoader**：BIM 格式批准后创建具体派生，通过 `SetBimLoader()` 注入；未注入时使用默认 `FakeBimLoader`，不虚构 BIM 要素（与 `AbstractMpcSolver` / `AbstractLocalizerBackend` 同理）。
- **FusionPose / SLAM 位姿**：上层节点通过 `OnFusionPose()` / `OnSlamPose()` 回调注入；位姿 lost 时不转换点云。
- **点云 / 检测事件**：由上层节点注入；未注入时标记 `valid=false`，不做位置匹配。

## 红线
- 位姿 lost 时不转换点云；栅格无数据源时 `valid=false`。
- 无法对应时保持「未定位」（`localized=false`，`coordinate_source="unlocalized"`），不伪造物理监测点。
- BIM 走抽象接口 `AbstractBimLoader`，不锁具体库（BIM 格式尚未获批）。

## 验证方式

- 纯逻辑测试（无需 ROS 2）：仓库根目录执行 `bash tools/reproduce.sh`（Linux/macOS）
  或 `powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1`（Windows），
  脚本依次运行 Python 单元测试与 C++ 冒烟测试；
- ROS 环境：`colcon build` 后 `ros2 launch inspection_bringup tech_1_7.launch.py`。
