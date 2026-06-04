#!/usr/bin/env python3

import argparse
from collections import defaultdict, deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


class WalkXZPlot(Node):
    def __init__(self, topic, foot_history_length):
        super().__init__("walk_xz_plot")
        self.create_subscription(MarkerArray, topic, self.marker_cb, 10)
        self.markers = defaultdict(dict)
        self.foot_history = defaultdict(lambda: deque(maxlen=foot_history_length))
        self.last_stamp = None

    def marker_cb(self, msg):
        self.markers.clear()
        for marker in msg.markers:
            self.markers[marker.ns][marker.id] = marker
            if marker.ns == "foot_current":
                point = marker.pose.position
                history = self.foot_history[marker.id]
                if not history or abs(history[-1][0] - point.x) > 1e-4 or abs(history[-1][1] - point.z) > 1e-4:
                    history.append((point.x, point.z))
        if msg.markers:
            self.last_stamp = msg.markers[0].header.stamp


def marker_points(marker):
    if marker is None:
        return []
    if marker.points:
        return [(p.x, p.z) for p in marker.points]
    return [(marker.pose.position.x, marker.pose.position.z)]


def plot_points(ax, points, *args, **kwargs):
    if not points:
        return
    xs, zs = zip(*points)
    ax.plot(xs, zs, *args, **kwargs)


def scatter_points(ax, points, *args, **kwargs):
    if not points:
        return
    xs, zs = zip(*points)
    ax.scatter(xs, zs, *args, **kwargs)


def main():
    parser = argparse.ArgumentParser(description="Plot Corgi walk debug markers in the X-Z plane.")
    parser.add_argument("--topic", default="/walk/debug_markers")
    parser.add_argument("--xlim", nargs=2, type=float, default=None)
    parser.add_argument("--zlim", nargs=2, type=float, default=[-0.08, 0.25])
    parser.add_argument("--history", action="store_true", help="Show touchdown history points.")
    parser.add_argument("--foot-history-length", type=int, default=600)
    parser.add_argument("--no-foot-history", action="store_true")
    parser.add_argument("--no-follow", action="store_true")
    parser.add_argument("--window", type=float, default=1.4, help="Initial x window width in meters.")
    args = parser.parse_args()

    rclpy.init()
    node = WalkXZPlot(args.topic, args.foot_history_length)

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.canvas.manager.set_window_title("Corgi Walk X-Z Debug")
    leg_colors = {
        40: "tab:red",
        41: "tab:green",
        42: "tab:blue",
        43: "gold",
    }
    view = {
        "initialized": False,
        "x_span": (args.xlim[1] - args.xlim[0]) if args.xlim is not None else args.window,
        "x_offset": 0.0,
        "z_span": args.zlim[1] - args.zlim[0],
        "z_center": 0.5 * (args.zlim[0] + args.zlim[1]),
        "last_com_x": 0.0,
        "follow": not args.no_follow,
    }

    def on_key(event):
        if event.key == "f":
            current_center = 0.5 * (ax.get_xlim()[0] + ax.get_xlim()[1])
            view["follow"] = not view["follow"]
            view["x_offset"] = current_center - view["last_com_x"] if view["follow"] else current_center
        elif event.key == "r":
            view["initialized"] = False
            view["x_span"] = (args.xlim[1] - args.xlim[0]) if args.xlim is not None else args.window
            view["z_span"] = args.zlim[1] - args.zlim[0]
            view["z_center"] = 0.5 * (args.zlim[0] + args.zlim[1])

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        rclpy.spin_once(node, timeout_sec=0.0)
        com = node.markers.get("com", {}).get(2)
        com_points = marker_points(com)
        com_x = com_points[0][0] if com_points else view["last_com_x"]

        if view["initialized"]:
            xlim = ax.get_xlim()
            zlim = ax.get_ylim()
            view["x_span"] = max(0.05, xlim[1] - xlim[0])
            view["z_span"] = max(0.05, zlim[1] - zlim[0])
            current_center = 0.5 * (xlim[0] + xlim[1])
            view["x_offset"] = current_center - view["last_com_x"] if view["follow"] else current_center
            view["z_center"] = 0.5 * (zlim[0] + zlim[1])
        elif args.xlim is not None:
            initial_center = 0.5 * (args.xlim[0] + args.xlim[1])
            view["x_offset"] = initial_center - com_x if view["follow"] else initial_center
            view["initialized"] = True
        else:
            view["x_offset"] = 0.0 if view["follow"] else com_x
            view["initialized"] = True

        view["last_com_x"] = com_x
        center_x = com_x + view["x_offset"] if view["follow"] else view["x_offset"]

        ax.clear()
        follow_status = "follow" if view["follow"] else "fixed"
        ax.set_title(f"Corgi Walk X-Z Projection ({follow_status}, press f toggle, r reset)")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("z [m]")
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="box")

        ax.set_xlim(center_x - 0.5 * view["x_span"], center_x + 0.5 * view["x_span"])
        ax.set_ylim(view["z_center"] - 0.5 * view["z_span"], view["z_center"] + 0.5 * view["z_span"])

        body = node.markers.get("body_outline", {}).get(0)
        plot_points(ax, marker_points(body), color="0.65", linewidth=2.0, label="body")

        hips = node.markers.get("hips", {}).get(1)
        scatter_points(ax, marker_points(hips), marker="s", s=55, color="0.75", label="hips")

        for marker in node.markers.get("touchdown_target", {}).values():
            scatter_points(ax, marker_points(marker), marker="o", s=60, alpha=0.65, label="touchdown target")

        if not args.no_foot_history:
            for marker_id, history in node.foot_history.items():
                plot_points(
                    ax,
                    list(history),
                    color=leg_colors.get(marker_id, "tab:gray"),
                    alpha=0.45,
                    linewidth=1.5,
                    label="current foot history")

        for marker in node.markers.get("foot_current", {}).values():
            color = leg_colors.get(marker.id, "tab:gray")
            scatter_points(ax, marker_points(marker), marker="x", s=60, linewidths=2.0, color=color, label="current foot")

        if args.history:
            for marker in node.markers.get("touchdown_history", {}).values():
                scatter_points(ax, marker_points(marker), marker=".", s=14, alpha=0.35, color="tab:blue")

        scatter_points(ax, marker_points(com), marker="*", s=150, color="white", edgecolors="black", label="CoM")

        handles, labels = ax.get_legend_handles_labels()
        unique = {}
        for handle, label in zip(handles, labels):
            unique.setdefault(label, handle)
        if unique:
            ax.legend(unique.values(), unique.keys(), loc="upper right")

    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    fig._walk_xz_animation = anim
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
