#!/usr/bin/env python3
"""odom_tf_relay: re-express a nav_msgs/Odometry into a different child frame.

Use-case
--------
FAST-LIO publishes /Odometry with child_frame_id = 'body' (the IMU centre).
corgi_fusion_node expects /lidar_odom whose child frame matches the leg-odom
output frame, i.e. 'base_link' (the robot centre).

The static transform T_{source←target} (e.g. T_{body←base_link}) is passed
directly as node parameters — no TF lookup is performed.  This avoids a TF
tree conflict: fast_lio broadcasts camera_init→body dynamically, and a second
static publisher for mid360_optical→body would give 'body' two parents, which
TF2 rejects.

Math applied on every incoming message:

    T_{world ← target} = T_{world ← source}  ⊗  T_{source ← target}

    p_target_world = p_source_world  +  R(q_source_world) @ t_target_in_source
    q_target_world = q_source_world  ⊗  q_source_from_target

Parameters
----------
input_topic   str    topic to subscribe (default '/Odometry')
output_topic  str    topic to publish   (default '/lidar_odom')
source_frame  str    child frame of the incoming odometry (default 'body')
target_frame  str    desired child frame of the output    (default 'base_link')

# Static T_{source←target}  (body ← base_link), pre-computed from:
#   base_link→mid360_optical: t=[0.1365,0,0.1881] RPY=[0.3491,0,1.5708]
#   mid360_optical→body:      t=[0.011,0.02329,-0.04412] RPY=[0,0,0]
static_t_x  float   x component of t_target_in_source  (default -0.010999)
static_t_y  float   y component of t_target_in_source  (default  0.040636)
static_t_z  float   z component of t_target_in_source  (default -0.179324)
static_q_w  float   w of q_source_from_target           (default  0.696361)
static_q_x  float   x of q_source_from_target           (default -0.122799)
static_q_y  float   y of q_source_from_target           (default -0.122800)
static_q_z  float   z of q_source_from_target           (default -0.696363)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


# ── Pure-numpy quaternion helpers ─────────────────────────────────────────────

def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product.  Both inputs and output are (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=np.float64)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate 3-vector v by unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    qv = np.array([x, y, z])
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


# ── Node ──────────────────────────────────────────────────────────────────────

class OdomTfRelay(Node):

    def __init__(self) -> None:
        super().__init__('odom_tf_relay')

        self.declare_parameter('input_topic',   '/Odometry')
        self.declare_parameter('output_topic',  '/lidar_odom')
        self.declare_parameter('source_frame',  'body')
        self.declare_parameter('target_frame',  'base_link')
        # T_{source←target} = T_{body←base_link}, pre-computed from CAD/hardware spec
        self.declare_parameter('static_t_x',  -0.010999)
        self.declare_parameter('static_t_y',   0.040636)
        self.declare_parameter('static_t_z',  -0.179324)
        self.declare_parameter('static_q_w',   0.696361)
        self.declare_parameter('static_q_x',  -0.122799)
        self.declare_parameter('static_q_y',  -0.122800)
        self.declare_parameter('static_q_z',  -0.696363)

        in_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.src_ = self.get_parameter('source_frame').get_parameter_value().string_value
        self.tgt_ = self.get_parameter('target_frame').get_parameter_value().string_value

        # Static T_{source←target}: position of target in source, rotation source←target
        tx = self.get_parameter('static_t_x').get_parameter_value().double_value
        ty = self.get_parameter('static_t_y').get_parameter_value().double_value
        tz = self.get_parameter('static_t_z').get_parameter_value().double_value
        qw = self.get_parameter('static_q_w').get_parameter_value().double_value
        qx = self.get_parameter('static_q_x').get_parameter_value().double_value
        qy = self.get_parameter('static_q_y').get_parameter_value().double_value
        qz = self.get_parameter('static_q_z').get_parameter_value().double_value

        self.t_tgt_in_src_   = np.array([tx, ty, tz])
        q = np.array([qw, qx, qy, qz])
        self.q_src_from_tgt_ = q / np.linalg.norm(q)  # ensure unit quaternion

        self.pub_ = self.create_publisher(Odometry, out_topic, 10)
        self.sub_ = self.create_subscription(Odometry, in_topic, self._cb, 10)

        self.get_logger().info(
            f'odom_tf_relay: {in_topic} (child={self.src_}) → '
            f'{out_topic} (child={self.tgt_}), '
            f't_tgt_in_src=[{tx:.4f},{ty:.4f},{tz:.4f}]')

    # ── Callback ──────────────────────────────────────────────────────────────

    def _cb(self, msg: Odometry) -> None:

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        p_src = np.array([p.x, p.y, p.z])
        q_src = np.array([o.w, o.x, o.y, o.z])   # (w, x, y, z)

        # T_{world ← target} = T_{world ← source}  ⊗  T_{source ← target}
        #
        #   p_tgt_world = p_src_world + R(q_src_world) @ t_tgt_in_src
        #   q_tgt_world = q_src_world ⊗ q_src_from_tgt
        #
        p_tgt = p_src + quat_rotate(q_src, self.t_tgt_in_src_)
        q_tgt = quat_mul(q_src, self.q_src_from_tgt_)
        q_tgt /= np.linalg.norm(q_tgt)

        out = Odometry()
        out.header         = msg.header          # same timestamp + world frame_id
        out.child_frame_id = self.tgt_
        out.pose.pose.position.x    = float(p_tgt[0])
        out.pose.pose.position.y    = float(p_tgt[1])
        out.pose.pose.position.z    = float(p_tgt[2])
        out.pose.pose.orientation.w = float(q_tgt[0])
        out.pose.pose.orientation.x = float(q_tgt[1])
        out.pose.pose.orientation.y = float(q_tgt[2])
        out.pose.pose.orientation.z = float(q_tgt[3])
        # Covariance is passed through unchanged (conservative)
        out.pose.covariance = msg.pose.covariance
        self.pub_.publish(out)


def main() -> None:
    rclpy.init()
    node = OdomTfRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
