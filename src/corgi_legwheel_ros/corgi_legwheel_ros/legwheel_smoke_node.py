#!/usr/bin/env python3
"""Smoke-test node for validating LegWheel import and basic FK call."""

from __future__ import annotations

import math
import sys
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node


class LegwheelSmokeNode(Node):
    """Run one-time import and FK checks against the external legwheel package."""

    def __init__(self) -> None:
        super().__init__("legwheel_smoke_node")

        self.declare_parameter("leg_index", 0)
        self.declare_parameter("theta_deg", 75.0)
        self.declare_parameter("beta_deg", 90.0)
        self.declare_parameter("gamma_deg", 0.0)

        self._timer = self.create_timer(0.2, self._run_once)
        self._done = False
        self.exit_code = 0

    @property
    def done(self) -> bool:
        return self._done

    def _run_once(self) -> None:
        if self._done:
            return

        self._done = True
        if self._timer is not None:
            self._timer.cancel()

        self.get_logger().info("Starting LegWheel smoke check...")

        try:
            import legwheel  # noqa: F401
            from legwheel.models.corgi_leg import CorgiLegKinematics
        except Exception as exc:  # pragma: no cover - runtime diagnostic path
            self.get_logger().error(
                "Cannot import legwheel package: "
                f"{exc}\n"
                "Install it before launching this node, e.g.\n"
                "  pip install -e /root/corgi_ws/corgi_ros2_ws/src/vendor/LegWheel"
            )
            self._shutdown_with_code(2)
            return

        leg_index = int(self.get_parameter("leg_index").value)
        theta_deg = float(self.get_parameter("theta_deg").value)
        beta_deg = float(self.get_parameter("beta_deg").value)
        gamma_deg = float(self.get_parameter("gamma_deg").value)

        try:
            leg = CorgiLegKinematics(leg_index)
            p_b: Optional[np.ndarray] = leg.forward_kinematics(
                math.radians(theta_deg),
                math.radians(beta_deg),
                math.radians(gamma_deg),
            )
            self.get_logger().info(
                "LegWheel OK | "
                f"leg={leg_index} theta={theta_deg:.1f} beta={beta_deg:.1f} "
                f"gamma={gamma_deg:.1f} -> p_B="
                f"{np.array2string(np.asarray(p_b), precision=4)}"
            )
            self._shutdown_with_code(0)
        except Exception as exc:  # pragma: no cover - runtime diagnostic path
            self.get_logger().error(f"LegWheel FK call failed: {exc}")
            self._shutdown_with_code(3)

    def _shutdown_with_code(self, code: int) -> None:
        self.exit_code = code
        if code == 0:
            self.get_logger().info("Smoke check completed successfully.")
        else:
            self.get_logger().error(f"Smoke check failed with code {code}.")
        self._done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LegwheelSmokeNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
