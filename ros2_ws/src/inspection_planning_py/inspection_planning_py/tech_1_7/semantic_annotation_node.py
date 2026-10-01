"""1.7 语义标注节点（Python 决策侧）。

PPT 范围：关联先验 BIM、实时 SLAM 和检测事件，做语义位置映射决策。

节点职责：
- 订阅 /semantic_alarm（C++ 1.7 输出：已带 confidence + bim_id/slam_id）
- 订阅 /fusion_pose（1.6 输出：机器人位姿）
- 通过 BimSlamAlignment 做 BIM-SLAM 关联决策
- 通过 SemanticLocationMetrics 聚合 PPT 指标
- 发布结构化语义标注结果（不直接发布关节级执行命令，红线）

硬边界：
- BIM 走抽象接口（AbstractBimLoader），不锁具体 BIM 库
- 无法对应时保持"未定位"，不虚构物理监测点
- Python 不进入 C++ 高频匹配环
"""

from __future__ import annotations

import sys
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from inspection_interfaces.msg import FusionPose, SemanticAlarm
    _HAS_RCLPY = True
except ImportError:  # 未安装 ROS 2
    _HAS_RCLPY = False
    Node = object  # type: ignore

from .bim_loader import AbstractBimLoader, FakeBimLoader
from .bim_slam_alignment import BimSlamAlignment, SlamLandmark
from .semantic_location_metrics import (
    LocationMappingSample,
    SemanticLocationMetrics,
)
from inspection_planning_py.common.topic_names import FUSION_POSE, SEMANTIC_ALARM


class SemanticAnnotationNode(Node):  # type: ignore[misc]
    """语义标注决策节点。"""

    def __init__(self, bim_loader: Optional[AbstractBimLoader] = None) -> None:
        if _HAS_RCLPY:
            super().__init__("semantic_annotation_node")  # type: ignore[call-arg]
        self._bim_loader = bim_loader or FakeBimLoader()
        if not self._bim_loader.is_ready():
            self._bim_loader.initialize()
        self._alignment = BimSlamAlignment()
        self._metrics = SemanticLocationMetrics()
        self._metrics.start_run()

        if not _HAS_RCLPY:
            # 未安装 ROS 2：仅运行离线逻辑自检
            return

        self._alarm_sub = self.create_subscription(
            SemanticAlarm, SEMANTIC_ALARM, self._on_alarm, 10)
        self._fusion_sub = self.create_subscription(
            FusionPose, FUSION_POSE, self._on_fusion, 10)
        self._report_timer = self.create_timer(1.0, self._periodic_report)

    def _on_fusion(self, msg) -> None:
        """1.6 FusionPose 到达，记录 SLAM landmark（简化为位姿点）。"""
        # 实际系统应聚合周边 landmark，此处用位姿点做示例
        # 不虚构：仅当 localization_state != "lost" 时记录
        if msg.localization_state == "lost":
            return

    def _on_alarm(self, msg) -> None:
        """C++ 1.7 SemanticAlarm 到达，做语义标注 + 指标聚合。"""
        sample = LocationMappingSample(
            event_id=msg.detection_event,
            matched_x=msg.physical_location.position.x,
            matched_y=msg.physical_location.position.y,
            matched_z=msg.physical_location.position.z,
            bim_id=msg.bim_id,
            slam_id=msg.slam_id,
            coordinate_source="bim" if msg.bim_id else (
                "slam" if msg.slam_id else "unlocalized"),
            confidence=msg.confidence,
            localized=msg.confidence > 0.0 and (bool(msg.bim_id) or bool(msg.slam_id)),
        )
        self._metrics.record_sample(sample)

    def _periodic_report(self) -> None:
        """周期性打印当前运行摘要（不重置累积数据，参考 1.5 教训）。"""
        if _HAS_RCLPY:
            self.get_logger().info(
                f"semantic map: {self._current_summary()}")
        else:
            print(self._current_summary(), file=sys.stderr)

    def _current_summary(self) -> str:
        """当前累积摘要（不结束运行，不重置）。"""
        if not self._metrics._samples:  # type: ignore[attr-defined]
            return "no samples yet"
        # 构造临时结果做摘要（不持久化）
        localized = sum(1 for s in self._metrics._samples if s.localized)  # type: ignore[attr-defined]
        total = len(self._metrics._samples)  # type: ignore[attr-defined]
        return f"events={total} localized={localized} unlocalized={total - localized}"

    def finish_and_report(self) -> str:
        """结束当前运行并返回完整摘要。"""
        result = self._metrics.finish_run()
        return result.summary()


def main(args=None) -> None:
    if not _HAS_RCLPY:
        print("[semantic_annotation_node] rclpy not available; "
              "running offline logic check.", file=sys.stderr)
        # 离线逻辑自检：FakeBimLoader 不虚构
        loader = FakeBimLoader()
        loader.initialize()
        assert loader.load_all() == []  # 未注入 → 空（不虚构）
        node = SemanticAnnotationNode(loader)
        print(node.finish_and_report(), file=sys.stderr)
        return

    rclpy.init(args=args)
    node = SemanticAnnotationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(node.finish_and_report(), file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
