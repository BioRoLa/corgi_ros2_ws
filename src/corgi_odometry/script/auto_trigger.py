#!/usr/bin/env python3
"""auto_trigger: publish a single TriggerStamped(enable=true) after a delay.

Used in corgi_odom_real.launch.py so that corgi_leg_odom starts automatically
without needing a manual ros2 topic pub command.

Parameters
----------
delay_sec  float  seconds to wait before publishing (default 3.0)
"""

import time
import rclpy
from rclpy.node import Node
from corgi_msgs.msg import TriggerStamped


class AutoTrigger(Node):

    def __init__(self) -> None:
        super().__init__('auto_trigger')
        self.declare_parameter('delay_sec', 8.0)
        delay = self.get_parameter('delay_sec').get_parameter_value().double_value
        self.pub_ = self.create_publisher(TriggerStamped, 'trigger', 1)
        self.timer_ = self.create_timer(delay, self._fire)
        self.get_logger().info(f'auto_trigger: will fire in {delay:.1f} s')

    def _fire(self) -> None:
        msg = TriggerStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.enable = True
        self.pub_.publish(msg)
        self.get_logger().info('auto_trigger: sent enable=true on /trigger')
        self.timer_.cancel()


def main() -> None:
    rclpy.init()
    node = AutoTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
