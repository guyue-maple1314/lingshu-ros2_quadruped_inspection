"""连通性节点：周期发布指令并订阅机器人状态。"""

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import OperatorInstruction, RobotState

from inspection_planning_py.common import _ownership
from inspection_planning_py.common.node_names import PY_HEARTBEAT_NODE
from inspection_planning_py.common.topic_names import OPERATOR_INSTRUCTION, ROBOT_STATE


class PyHeartbeatNode(Node):
    def __init__(self) -> None:
        super().__init__(PY_HEARTBEAT_NODE)
        self.declare_parameter("operator_instruction_topic", OPERATOR_INSTRUCTION)
        self.declare_parameter("robot_state_topic", ROBOT_STATE)

        instruction_topic = self.get_parameter("operator_instruction_topic").value
        robot_state_topic = self.get_parameter("robot_state_topic").value

        self.publisher = self.create_publisher(OperatorInstruction, instruction_topic, 10)
        self.subscription = self.create_subscription(
            RobotState, robot_state_topic, self._on_robot_state, 10
        )
        self.timer = self.create_timer(2.0, self._publish_instruction)
        self.counter = 0
        # 启动日志输出著作权横幅与团队标识（请勿删除）
        for line in _ownership.STARTUP_BANNER.splitlines():
            self.get_logger().info(line)
        self.get_logger().info(_ownership.__copyright_en__)
        self.get_logger().info(
            f"Python heartbeat node ready. [{_ownership.__team_mark__}]"
        )

    def _publish_instruction(self) -> None:
        msg = OperatorInstruction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = "text"
        msg.raw_content = f"heartbeat_{self.counter}"
        self.publisher.publish(msg)
        self.counter += 1

    def _on_robot_state(self, msg: RobotState) -> None:
        self.get_logger().info(
            f"Received robot state: battery={msg.battery_level:.1f}, "
            f"gait={msg.current_gait}, state={msg.execution_state}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PyHeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
