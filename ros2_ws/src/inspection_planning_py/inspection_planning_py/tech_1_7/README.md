# tech_1_7 语义地图（Python 决策侧）

## 四模块
- `bim_loader`：读取项目方批准的 BIM 格式
  - `AbstractBimLoader` 抽象接口 + `FakeBimLoader` 确定性实现（BIM 格式未获批，不锁具体库，不虚构 BIM 要素）
- `bim_slam_alignment`：BIM 先验与实时 SLAM 关联
  - 30cm 半径关联，位置结果带 `confidence` + `coordinate_source`（`bim_id`/`slam_id`/`unlocalized`）
- `semantic_annotation_node`：检测事件映射到语义位置（ROS 节点，离线可运行）
  - 无法对应时保持「未定位」（`localized=false`），不虚构物理监测点（红线）
- `semantic_location_metrics`：厘米级映射与位置报告准确率
  - PPT 阈值：语义位置映射精度 5cm（`SEMANTIC_MAPPING_TOLERANCE_M`），位置报告准确率 ≥98%（`PPT_LOCATION_REPORT_ACCURACY`）

## 红线
- BIM 走抽象接口 `AbstractBimLoader`（与 `AbstractMpcSolver` / `AbstractLocalizerBackend` 同理）。
- 位置结果带 `confidence` + `bim_id`/`slam_id`（坐标来源可追溯）。
- 无法对应时保持「未定位」（`localized=false`，`coordinate_source="unlocalized"`），不虚构物理监测点。
- 离线/在线分离；Python 用 `unittest`（非 pytest）。

## 验证方式

- 纯逻辑测试（无需 ROS 2）：仓库根目录执行 `bash tools/reproduce.sh`（Linux/macOS）
  或 `powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1`（Windows），
  脚本依次运行 Python 单元测试与 C++ 冒烟测试；
- ROS 环境：`colcon build` 后 `ros2 launch inspection_bringup tech_1_7.launch.py`。
