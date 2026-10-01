# 技术 1.7 验收（高精度时空融合语义地图构建与定位）

## 硬边界
- C++ 侧点云转换 / 栅格赋权 / 位置匹配在节点内运行，Python 决策侧不阻塞地图构建回路。
- BIM 格式尚未获批：`AbstractBimLoader` 抽象接口，`FakeBimLoader` 仅做确定性 BIM 要素输出，不虚构 BIM 数据。
- 位置结果带 `confidence` + `coordinate_source`（`bim_id` / `slam_id` / `unlocalized`），无法对应时保持「未定位」（`localized=false`），不输出虚假物理监测点（红线）。
- 位姿 lost 时 `PointcloudTransformer` 不转换点云；`WeightedGridBuilder` 无数据源时 `valid=false`。

## adapter / 注入说明
`tech_1_7_node` 内部四组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **AbstractBimLoader**：BIM 格式批准后创建具体派生，通过 `SetBimLoader()` 注入；未注入时使用默认 `FakeBimLoader`，不虚构 BIM 要素。
- **FusionPose / SLAM 位姿**：由上层节点启动时通过 `OnFusionPose()` / `OnSlamPose()` 回调注入；位姿 lost 时不转换点云。
- **点云 / 检测事件**：由上层注入，未接入时标记 `valid=false`，不做位置匹配。
- **SemanticAlarm**：复用已有消息接口，由 `AlarmLocationPublisher` 发布，不新增消息类型。

## PPT 指标（阈值已落地 `semantic_location_metrics.py`）
- 语义位置映射精度 **5cm**（0.05m，`SEMANTIC_MAPPING_TOLERANCE_M`）
- 位置报告准确率 ≥ **98%**（`PPT_LOCATION_REPORT_ACCURACY`）

## 组件架构
```
PointcloudTransformer → WeightedGridBuilder
→ PositionMatcher（BIM-SLAM 关联，30cm 半径）
→ AlarmLocationPublisher（发布 SemanticAlarm）
```
- **PointcloudTransformer**：点云坐标转换，位姿 lost 时不输出
- **WeightedGridBuilder**：栅格赋权，无数据源时 valid=false
- **PositionMatcher**：BIM 先验与实时 SLAM 关联，半径 30cm，带 confidence + 坐标来源
- **AlarmLocationPublisher**：检测事件映射到语义位置，发布 SemanticAlarm

## 必测场景
详见 [scenarios.md](./scenarios.md)。

## 证据模板
验收证据按本目录 `scenarios.md` 的场景逐项归档。

## 验证方式

- 纯逻辑测试（无需 ROS 2）：仓库根目录执行 `bash tools/reproduce.sh`（Linux/macOS）
  或 `powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1`（Windows），
  脚本依次运行 Python 单元测试与 C++ 冒烟测试；
- ROS 环境：`colcon build` 后 `ros2 launch inspection_bringup tech_1_7.launch.py`。
