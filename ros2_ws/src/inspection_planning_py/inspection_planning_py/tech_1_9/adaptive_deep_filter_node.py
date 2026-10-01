"""1.9 自适应深度滤波节点：抽象深度滤波接口 + 本地桩实现。"""

from abc import ABC, abstractmethod
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import AcousticDiagnosis

from inspection_planning_py.common.node_names import ADAPTIVE_DEEP_FILTER_NODE
from inspection_planning_py.common.topic_names import ACOUSTIC_DIAGNOSIS


class AbstractDeepFilter(ABC):
    @abstractmethod
    def filter(self, audio: List[float]) -> Tuple[bool, List[float]]:
        """返回 (ok, filtered_audio)。"""


class FakeDeepFilter(AbstractDeepFilter):
    def filter(self, audio: List[float]) -> Tuple[bool, List[float]]:
        # 不输出未经后端处理的滤波效果：原样返回
        return (True, list(audio))


class AdaptiveDeepFilterNode(Node):
    def __init__(self) -> None:
        super().__init__(ADAPTIVE_DEEP_FILTER_NODE)
        self._filter: AbstractDeepFilter = FakeDeepFilter()
        # 先订阅 AcousticDiagnosis 接收波束与 SNR 状态
        self.create_subscription(AcousticDiagnosis, ACOUSTIC_DIAGNOSIS, self._on_diagnosis, 10)

    def _on_diagnosis(self, msg: AcousticDiagnosis) -> None:
        self.get_logger().info(
            f"deep filter received snr={msg.output_snr_db:.1f}dB beam={msg.beam_azimuth:.1f}deg"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdaptiveDeepFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
