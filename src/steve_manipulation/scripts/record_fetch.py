#!/usr/bin/env python3
"""Record a live fetch or whole-body pick and draw /odom plus joint velocities.

Start this before the task, in the same ROS graph as Gazebo. It waits for
/fetch/phase (or /wbc/phase), then stores /odom and the six arm speeds from
/joint_states until the task publishes Done or Failed. Ctrl-C saves whatever
has arrived.

  ros2 launch steve_manipulation manipulation.launch.py run_fetch:=true record:=true

  # Gazebo already running:
  ros2 run steve_manipulation record_fetch.py --output ~/steve_ws/docs/fetch_cube/live

Redraw a saved run without ROS:

  python3 scripts/record_fetch.py --plot docs/fetch_cube/live/fetch_YYYYMMDD_HHMMSS.npz
"""

import argparse
import os
import sys
from datetime import datetime

import numpy as np

ARM = (
    "ur5eshoulder_pan_joint",
    "ur5eshoulder_lift_joint",
    "ur5eelbow_joint",
    "ur5ewrist_1_joint",
    "ur5ewrist_2_joint",
    "ur5ewrist_3_joint",
)
SHORT = ("shoulder pan", "shoulder lift", "elbow", "wrist 1", "wrist 2", "wrist 3")
DONE = {"done", "failed"}
PHASE_COLOR = {
    "Fold home": "#e7e7e7",
    "See cube": "#d9e2f2",
    "Drive to stand": "#f7e3c4",
    "Step back": "#f5e6cc",
    "Measure": "#d8efe2",
    "Remeasure": "#d8efe2",
    "Pregrasp": "#f3d6d6",
    "Descend": "#e6dcf3",
    "Back off": "#fdebd0",
    "Close": "#f6d6e8",
    "Lift": "#d7eef4",
    "Recover": "#f5b7b1",
    "Back to start": "#dcead4",
    "Drive to table": "#f8d7d0",
    "Above table": "#ddd6f3",
    "Lower": "#d6eaf8",
    "Release": "#fdebd0",
    "Retreat": "#f5e6cc",
    "Approach": "#f7e3c4",
    "At grasp": "#f6d6e8",
    "Grasp": "#f6d6e8",
    "Rise": "#fdebd0",
    "Carry": "#dcead4",
    "Carry home": "#dcead4",
}
FALLBACK = ["#d9e2f2", "#f7e3c4", "#d8efe2", "#f3d6d6", "#e6dcf3", "#d7eef4", "#f6efc6"]


def _yaw(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _label(name):
    text = name.strip()
    if not text:
        return ""
    if text[:1].islower():
        return text[:1].upper() + text[1:]
    return text


def bands(times, names):
    grouped = []
    for t, name in zip(times, names):
        if not name:
            continue
        if grouped and grouped[-1][2] == name:
            grouped[-1] = (grouped[-1][0], t, name)
        else:
            grouped.append((t, t, name))
    if grouped:
        grouped[-1] = (grouped[-1][0], times[-1], grouped[-1][2])
    return grouped


def save_run(path, rows):
    data = np.array([row[:13] for row in rows], dtype=float)
    phases = np.array([row[13] for row in rows])
    np.savez(path, data=data, phase=phases)
    return path


def load_run(path):
    saved = np.load(path, allow_pickle=True)
    data = saved["data"]
    phase = [_label(str(name)) for name in saved["phase"]]
    return data, phase


def draw(data, phase, title, out_png):
    import matplotlib.pyplot as plt

    t = data[:, 0] - data[0, 0]
    speed = np.hypot(data[:, 4], data[:, 5])
    qd = data[:, 7:13]
    spans = bands(t, phase)

    fig, axes = plt.subplots(
        4,
        1,
        figsize=(13.5, 8.6),
        sharex=True,
        gridspec_kw={"height_ratios": [0.55, 1.15, 0.85, 1.4], "hspace": 0.16},
    )
    ribbon, axes = axes[0], axes[1:]
    fig.suptitle(title, fontsize=13, y=0.98)

    axes[0].plot(t, data[:, 1], color="#1f4e79", lw=1.6, label="odom x")
    axes[0].plot(t, data[:, 2], color="#c45911", lw=1.6, label="odom y")
    axes[0].set_ylabel("position (m)")
    axes[0].legend(loc="upper left", frameon=False, ncol=2)
    axes[0].set_title("/odom  pose in the odom frame", loc="left", fontsize=10)

    axes[1].plot(t, speed, color="#1f4e79", lw=1.6, label="planar speed")
    axes[1].plot(t, data[:, 6], color="#548235", lw=1.4, label="yaw rate")
    axes[1].set_ylabel("m/s, rad/s")
    axes[1].legend(loc="upper right", frameon=False, ncol=2)
    axes[1].set_title("/odom  twist of the base", loc="left", fontsize=10)

    colors = ["#1f4e79", "#c45911", "#548235", "#833c0c", "#5b2c6f", "#1a5276"]
    for index, name in enumerate(SHORT):
        axes[2].plot(t, qd[:, index], color=colors[index], lw=1.15, label=name)
    axes[2].set_ylabel("rad/s")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right", frameon=False, ncol=3, fontsize=8)
    axes[2].set_title("arm joint velocities  (six UR5e joints, from /joint_states)", loc="left", fontsize=10)

    for index, (t0, t1, name) in enumerate(spans):
        color = PHASE_COLOR.get(name, FALLBACK[index % len(FALLBACK)])
        for axis in axes:
            axis.axvspan(t0, t1, color=color, lw=0, zorder=0)
        ribbon.axvspan(t0, t1, color=color, lw=0)
        ribbon.text(
            0.5 * (t0 + t1),
            0.5,
            name,
            ha="center",
            va="center",
            fontsize=7.5,
            color="#222222",
            rotation=90 if (t1 - t0) < 3.2 else 0,
            clip_on=True,
        )
    ribbon.set_ylim(0, 1)
    ribbon.set_yticks([])
    ribbon.set_title("phases reported by the running node", loc="left", fontsize=10)
    for spine in ribbon.spines.values():
        spine.set_visible(False)
    for axis in list(axes) + [ribbon]:
        axis.set_xlim(t[0], t[-1])
    for axis in axes:
        axis.grid(True, color="#e6e6e6", lw=0.6)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)

    fig.text(
        0.07,
        0.012,
        "Source: Gazebo topics /odom and /joint_states, with the phase topic from the task. Sim time, starting at the first phase.",
        fontsize=8,
        color="#555555",
    )
    fig.subplots_adjust(left=0.07, right=0.98, top=0.91, bottom=0.08)
    fig.savefig(out_png, dpi=140, bbox_inches="tight", facecolor="white")
    print(out_png)


def record(args):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    phase_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    class Recorder(Node):
        def __init__(self):
            super().__init__(
                "record_fetch",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            )
            self.phase = ""
            self.started = False
            self.finished = False
            self._warned_phase = False
            self.rows = []
            self.joints = None
            self.joint_stamp = None
            self.prev_q = None
            self.prev_stamp = None
            self.create_subscription(String, args.phase_topic, self._on_phase, phase_qos)
            self.create_subscription(Odometry, "/odom", self._on_odom, 50)
            self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
            self.get_logger().info(
                f"Waiting for {args.phase_topic}. Recording /odom and /joint_states until Done or Failed."
            )

        def _on_phase(self, msg: String):
            self.phase = _label(msg.data)
            self.started = True
            if msg.data.strip().lower() in DONE:
                self.finished = True

        def _on_joints(self, msg: JointState):
            by_name = dict(zip(msg.name, msg.position))
            if not all(name in by_name for name in ARM):
                return
            q = np.array([by_name[name] for name in ARM])
            stamp = _seconds(msg.header.stamp)
            reported = None
            if len(msg.velocity) == len(msg.name):
                by_vel = dict(zip(msg.name, msg.velocity))
                if all(name in by_vel for name in ARM):
                    reported = np.array([by_vel[name] for name in ARM], dtype=float)
            derived = None
            if self.prev_q is not None and stamp > self.prev_stamp + 1e-4:
                derived = (q - self.prev_q) / (stamp - self.prev_stamp)
            # Some hardware fills velocity with zeros. A real move still shows up in position.
            if reported is None or (
                derived is not None
                and np.max(np.abs(reported)) < 1e-4
                and np.max(np.abs(derived)) > 0.02
            ):
                reported = derived
            self.prev_q = q
            self.prev_stamp = stamp
            self.joints = reported
            self.joint_stamp = stamp

        def _on_odom(self, msg: Odometry):
            if not self.started:
                if not self._warned_phase:
                    self.get_logger().warn(
                        f"/odom is arriving, but {args.phase_topic} has not. "
                        "Nothing is written until that phase topic connects."
                    )
                    self._warned_phase = True
                return
            if self.joints is None:
                return
            stamp = _seconds(msg.header.stamp)
            pose = msg.pose.pose.position
            twist = msg.twist.twist
            self.rows.append(
                (
                    stamp,
                    pose.x,
                    pose.y,
                    float(_yaw(msg.pose.pose.orientation)),
                    twist.linear.x,
                    twist.linear.y,
                    twist.angular.z,
                    *self.joints,
                    self.phase,
                )
            )

    rclpy.init(args=sys.argv)
    node = Recorder()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped. Saving the samples collected so far.")
    finally:
        rows = node.rows
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if len(rows) < 2:
        print("No samples recorded. The phase topic never arrived, or /odom and /joint_states were quiet.")
        return 1
    os.makedirs(args.output, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = "wbc" if args.phase_topic.endswith("wbc/phase") else "fetch"
    npz = os.path.join(args.output, f"{prefix}_{stamp}.npz")
    png = os.path.join(args.output, f"{prefix}_{stamp}.png")
    save_run(npz, rows)
    print(npz)
    data, phase = load_run(npz)
    draw(data, phase, args.title, png)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=os.getcwd(), help="Directory for the npz and the png")
    parser.add_argument("--phase-topic", default="/fetch/phase")
    parser.add_argument("--title", default="fetch_cube.py  —  live Gazebo run")
    parser.add_argument("--plot", metavar="NPZ", help="Draw an already saved run and exit")
    args, _unknown = parser.parse_known_args(argv)
    if args.plot:
        data, phase = load_run(args.plot)
        png = os.path.splitext(args.plot)[0] + ".png"
        draw(data, phase, args.title, png)
        return 0
    return record(args)


if __name__ == "__main__":
    sys.exit(main() or 0)
