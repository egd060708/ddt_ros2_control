#!/usr/bin/env python3

from __future__ import annotations

import os
import sys

# 多个 Robot 使用 controller "<extern>" 时，必须指定连接哪一个。
# Webots 使用环境变量 WEBOTS_CONTROLLER_URL：可设为「仅机器人名」（无 ipc://），
# 运行库会自动补全为 ipc://<实例端口>/<name>（与 webots-controller --robot-name= 一致）。
def _apply_webots_extern_target_from_argv() -> None:
    for i, arg in enumerate(sys.argv):
        if arg.startswith("--robot-name="):
            os.environ["WEBOTS_CONTROLLER_URL"] = arg.split("=", 1)[1]
            return
        if arg == "--robot-name" and i + 1 < len(sys.argv):
            os.environ["WEBOTS_CONTROLLER_URL"] = sys.argv[i + 1]
            return


_apply_webots_extern_target_from_argv()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# webots controller Python API（须在设置 WEBOTS_CONTROLLER_URL 之后导入）
from controller import Supervisor


class TranslationSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("translation_supervisor")
        self.declare_parameter("tracked_robot_name", "d1h")
        self._tracked_robot_name = (
            self.get_parameter("tracked_robot_name").get_parameter_value().string_value
        )
        self._robot = Supervisor()
        self._time_step = int(self._robot.getBasicTimeStep())
        self._tracked_node = None
        self._pub = self.create_publisher(Float64, "/body_height", 10)
        self.get_logger().info(
            f"TranslationSupervisor started, tracking '{self._tracked_robot_name}'"
        )

    def _resolve_tracked_node(self):
        node = self._robot.getFromDef(self._tracked_robot_name)
        if node is not None:
            return node

        root = self._robot.getRoot()
        if root is None:
            return None
        children = root.getField("children")
        if children is None:
            return None

        for i in range(children.getCount()):
            candidate = children.getMFNode(i)
            if candidate is None:
                continue
            name_field = candidate.getField("name")
            if name_field is not None and name_field.getSFString() == self._tracked_robot_name:
                return candidate
        return None

    def step_and_publish(self) -> bool:
        if self._robot.step(self._time_step) < 0:
            return False

        if self._tracked_node is None:
            self._tracked_node = self._resolve_tracked_node()
            if self._tracked_node is None:
                return True
            self.get_logger().info(
                f"Resolved tracked robot '{self._tracked_robot_name}', publishing /body_height"
            )

        position = self._tracked_node.getPosition()
        if position is None or len(position) < 3:
            return True

        msg = Float64()
        msg.data = float(position[2])
        self._pub.publish(msg)
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TranslationSupervisorNode()
    try:
        while rclpy.ok():
            if not node.step_and_publish():
                break
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
