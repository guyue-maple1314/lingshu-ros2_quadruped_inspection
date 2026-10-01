"""1.8 红外测温异常高温判定节点（Python 决策侧）。

PPT 范围：基于有效补偿温度（in_range）输出异常高温判定。

节点职责：
- 订阅 /thermal_measurement（C++ 1.8 输出：补偿温度 + error_state）
- 仅在 error_state == "ok"（范围内）时做异常高温判定
- 发布 /inspection_alert（复用已有消息，不新增类型）
- 通过 ThermalMetrics 聚合 PPT 指标

硬边界：
- 超范围（error_state != "ok"）时不做异常高温判定
  （不按范围内精度发布，红线）
- 稳像与温度补偿在 C++ 执行，Python 不重复实现
- 不虚构温度/精度
"""

from __future__ import annotations

import sys
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from inspection_interfaces.msg import ThermalMeasurement, InspectionAlert
    _HAS_RCLPY = True
except ImportError:  # 未安装 ROS 2
    _HAS_RCLPY = False
    Node = object  # type: ignore

from ..common.node_names import THERMAL_ANOMALY_DECISION_NODE
from ..common.topic_names import THERMAL_MEASUREMENT, INSPECTION_ALERT
from ..common import error_codes
from .thermal_metrics import (
    ThermalMeasurementSample,
    ThermalMetrics,
)

# 异常高温判定阈值（由项目方按现场标定）
ANOMALY_HIGH_TEMP_C = 80.0


class ThermalAnomalyDecisionNode(Node):  # type: ignore[misc]
    """红外测温异常高温判定节点。"""

    def __init__(self,
                 anomaly_threshold_c: float = ANOMALY_HIGH_TEMP_C) -> None:
        if _HAS_RCLPY:
            super().__init__(THERMAL_ANOMALY_DECISION_NODE)  # type: ignore[call-arg]
        self._threshold = anomaly_threshold_c
        self._metrics = ThermalMetrics()
        self._metrics.start_run()

        if not _HAS_RCLPY:
            # 未安装 ROS 2：仅运行离线逻辑自检
            return

        self._thermal_sub = self.create_subscription(
            ThermalMeasurement, THERMAL_MEASUREMENT,
            self._on_thermal, 10)
        self._alert_pub = self.create_publisher(
            InspectionAlert, INSPECTION_ALERT, 10)
        self._report_timer = self.create_timer(1.0, self._periodic_report)

    def _on_thermal(self, msg) -> None:
        """C++ 1.8 ThermalMeasurement 到达，做异常高温判定。"""
        in_range = (msg.error_state == "ok")
        sample = ThermalMeasurementSample(
            raw_temperature=msg.raw_temperature,
            compensated_temperature=msg.compensated_temperature,
            angle=msg.angle,
            distance=msg.distance,
            emissivity=msg.emissivity,
            correction_factor=msg.correction_factor,
            error_state=msg.error_state,
            in_range=in_range,
        )
        self._metrics.record_sample(sample)

        # 红线：超范围（error_state != "ok"）时不做异常高温判定
        if not in_range:
            self.get_logger().debug(
                f"skip anomaly decision: {error_codes.describe(error_codes.THERMAL_OUT_OF_RANGE)}"
                f" ({msg.error_state})")
            return

        # 范围内 → 异常高温判定
        if msg.compensated_temperature > self._threshold:
            alert = InspectionAlert()
            alert.header.stamp = msg.header.stamp
            alert.alert_type = "thermal_anomaly"
            alert.source_device = "thermal_1_8"
            alert.detected_value = msg.compensated_temperature
            # 本节点拿不到目标相对位置（无方位信息），显式标记位姿无效：
            # 由 1.7 判定为"未定位"，不得用默认零位姿做位置匹配（红线）
            alert.pose_valid = False
            self._alert_pub.publish(alert)

    def _periodic_report(self) -> None:
        """周期性打印当前运行摘要。"""
        if _HAS_RCLPY:
            self.get_logger().info(
                f"thermal: {self._current_summary()}")
        else:
            print(self._current_summary(), file=sys.stderr)

    def _current_summary(self) -> str:
        """当前累积摘要。"""
        if not self._metrics._samples:  # type: ignore[attr-defined]
            return "no samples yet"
        in_range = sum(1 for s in self._metrics._samples if s.in_range)  # type: ignore[attr-defined]
        total = len(self._metrics._samples)  # type: ignore[attr-defined]
        return (f"events={total} in_range={in_range} "
                f"out_of_range={total - in_range}")

    def finish_and_report(self) -> str:
        """结束当前运行并返回完整摘要。"""
        result = self._metrics.finish_run()
        return result.summary()


def main(args=None) -> None:
    if not _HAS_RCLPY:
        print("[thermal_anomaly_decision_node] rclpy not available; "
              "running offline logic check.", file=sys.stderr)
        # 离线逻辑自检：超范围样本不判定异常高温
        node = ThermalAnomalyDecisionNode()
        # 模拟超范围样本
        out_sample = ThermalMeasurementSample(
            compensated_temperature=200.0,
            error_state="angle_out_of_range",
            in_range=False,
        )
        node._metrics.record_sample(out_sample)
        print(node.finish_and_report(), file=sys.stderr)
        return

    rclpy.init(args=args)
    node = ThermalAnomalyDecisionNode()
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
