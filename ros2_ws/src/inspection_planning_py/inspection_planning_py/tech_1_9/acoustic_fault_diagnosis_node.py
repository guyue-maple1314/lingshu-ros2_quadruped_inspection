"""1.9 声纹故障诊断节点：用抽象模型对波束形成后的音频做分类并回填故障类别。

数据流：
- 订阅 /acoustic_mono（AcousticFrame）：C++ 1.9 波束形成后的单通道波形；
- 订阅 /acoustic_diagnosis（AcousticDiagnosis）：C++ 1.9 的波束方位 / SNR 状态；
- 分类结果回填 fault_class 后仍发布到 /acoustic_diagnosis（与 C++ 一致的消息类型）。

边界（红线）：
- 只处理 fault_class 为空的消息：本节点自己发布的分类结果带类别，不会再次触发
  推理，避免同话题自激循环。
- 未装载模型、或没有新鲜音频帧时不做推理，不回填、不虚构故障类别。
- 判定为故障时另发 /inspection_alert（pose_valid=false）：本节点拿不到目标相对
  位置，1.7 据此判「未定位」，不得据此编造物理监测点。
- 音频波形由 C++ 侧合成示例生成，未接真实麦克风阵列前不得引用为实测结论。
"""

from __future__ import annotations

import sys
from typing import List, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from inspection_interfaces.msg import (
        AcousticDiagnosis,
        AcousticFrame,
        InspectionAlert,
    )
    _HAS_RCLPY = True
except ImportError:  # 未安装 ROS 2
    _HAS_RCLPY = False
    Node = object  # type: ignore

from inspection_planning_py.common.node_names import ACOUSTIC_FAULT_DIAGNOSIS_NODE
from inspection_planning_py.common.topic_names import (
    ACOUSTIC_DIAGNOSIS,
    ACOUSTIC_MONO,
    INSPECTION_ALERT,
)
from inspection_planning_py.common import error_codes
from inspection_planning_py.tech_1_9.acoustic_metrics import AcousticMetrics
from inspection_planning_py.tech_1_9.acoustic_model_loader import (
    AbstractAcousticModel,
    FakeAcousticModel,
)

#: 音频帧最大时延（秒）：超过判为过期，不用过期波形推理
DEFAULT_AUDIO_MAX_AGE_SEC = 1.0
#: 判定为故障并对外告警所需的最低置信度
DEFAULT_FAULT_CONFIDENCE_THRESHOLD = 0.5


class AcousticFaultDiagnosisNode(Node):  # type: ignore[misc]
    """声纹故障诊断节点（纯逻辑部分可离线单测）。"""

    def __init__(self,
                 model: Optional[AbstractAcousticModel] = None,
                 confidence_threshold: float = DEFAULT_FAULT_CONFIDENCE_THRESHOLD,
                 audio_max_age_sec: float = DEFAULT_AUDIO_MAX_AGE_SEC) -> None:
        if _HAS_RCLPY:
            super().__init__(ACOUSTIC_FAULT_DIAGNOSIS_NODE)  # type: ignore[call-arg]

        self._confidence_threshold = float(confidence_threshold)
        self._audio_max_age_sec = float(audio_max_age_sec)
        self._metrics = AcousticMetrics()
        self._audio_samples: List[float] = []
        self._audio_stamp_sec: float = 0.0

        if not _HAS_RCLPY:
            self._model = model or FakeAcousticModel()
            return

        self.declare_parameter("model_path", "")
        self.declare_parameter("fault_confidence_threshold",
                               DEFAULT_FAULT_CONFIDENCE_THRESHOLD)
        self.declare_parameter("audio_max_age_sec", DEFAULT_AUDIO_MAX_AGE_SEC)
        self._confidence_threshold = float(
            self.get_parameter("fault_confidence_threshold").value)
        self._audio_max_age_sec = float(
            self.get_parameter("audio_max_age_sec").value)

        self._model = model or FakeAcousticModel()
        model_path = self.get_parameter("model_path").value
        if model_path:
            self._model.load(model_path)

        self._publisher = self.create_publisher(
            AcousticDiagnosis, ACOUSTIC_DIAGNOSIS, 10)
        self._alert_pub = self.create_publisher(
            InspectionAlert, INSPECTION_ALERT, 10)
        self.create_subscription(
            AcousticFrame, ACOUSTIC_MONO, self._on_audio, 10)
        self.create_subscription(
            AcousticDiagnosis, ACOUSTIC_DIAGNOSIS, self._on_diagnosis, 10)

    # ---- 音频帧缓存（供模型推理使用）----
    def _on_audio(self, msg) -> None:
        if not msg.valid:
            return
        stamp = msg.header.stamp
        self._audio_stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self._audio_samples = [float(v) for v in msg.samples]

    def _audio_is_fresh(self, now_sec: float) -> bool:
        if not self._audio_samples:
            return False
        age = now_sec - self._audio_stamp_sec
        # 时间戳缺失（0）时不判过期，交由模型处理；有戳则按时限校验
        if self._audio_stamp_sec <= 0.0:
            return True
        return 0.0 <= age <= self._audio_max_age_sec

    # ---- 纯逻辑：可离线单测，不依赖 rclpy ----
    def _diagnose(self, msg, now_sec: float = 0.0
                  ) -> Optional[Tuple[str, float]]:
        """返回 (fault_class, confidence)；不应推理时返回 None。"""
        # 防自激：已带类别的消息（含本节点自己发布的）不再推理
        if msg.fault_class:
            return None
        # 未装载模型 → 不推理，不虚构类别
        if not self._model.is_loaded():
            return None
        # 没有新鲜音频 → 不推理（模型输入不能为空）
        if not self._audio_is_fresh(now_sec):
            return None
        ok, fault_class, confidence = self._model.infer(list(self._audio_samples))
        if not ok or not fault_class:
            return None
        return fault_class, float(confidence)

    # ---- ROS 回调 ----
    def _on_diagnosis(self, msg) -> None:
        self._metrics.record_snr(msg.output_snr_db)
        now_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        decision = self._diagnose(msg, now_sec)
        if decision is None:
            self.get_logger().debug(
                f"snr={msg.output_snr_db:.1f}dB "
                f"({error_codes.describe(error_codes.ACOUSTIC_LOW_CONFIDENCE)}: no valid diagnosis)"
            )
            return

        fault_class, confidence = decision
        out = AcousticDiagnosis()
        out.header = msg.header
        out.beam_azimuth = msg.beam_azimuth
        out.beam_elevation = msg.beam_elevation
        out.background_noise_db = msg.background_noise_db
        out.output_snr_db = msg.output_snr_db
        out.fault_class = fault_class
        out.confidence = confidence
        self._publisher.publish(out)
        self.get_logger().info(
            f"fault={fault_class} confidence={confidence:.2f}"
        )

        if confidence >= self._confidence_threshold:
            alert = InspectionAlert()
            alert.header = msg.header
            alert.alert_type = "acoustic_fault"
            alert.source_device = "acoustic_1_9"
            alert.detected_value = float(msg.output_snr_db)
            alert.pose_valid = False  # 无目标相对位置，1.7 判未定位（红线）
            self._alert_pub.publish(alert)


def main(args=None) -> None:
    if not _HAS_RCLPY:
        print("[acoustic_fault_diagnosis_node] rclpy not available; "
              "running offline logic check.", file=sys.stderr)
        # 离线自检：无音频时不推理（不虚构故障类别）
        node = AcousticFaultDiagnosisNode()
        class _Msg:  # 最小替身，避免依赖 ROS 消息类
            fault_class = ""
        print(f"offline diagnose → {node._diagnose(_Msg(), 0.0)}", file=sys.stderr)
        return

    rclpy.init(args=args)
    node = AcousticFaultDiagnosisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
