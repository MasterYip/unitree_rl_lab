#!/usr/bin/env python3
"""
Real-time G1-29dof Robot State Visualizer
=========================================

Connects to the G1 robot via DDS (``unitree_sdk2_py``) and displays live
joint positions, velocities, IMU orientation, and system status in a
matplotlib GUI.

Usage::

    python vis_g1_state.py --network enp5s0

    # On HiDPI / 4K screens use a scale factor:
    python vis_g1_state.py --network enp5s0 --scale 1.5
    python vis_g1_state.py --network enp5s0 -s 2.0

    # Or if unitree_sdk2_py is installed in the default environment:
    python vis_g1_state.py --network lo   # loopback (simulation)

Requirements
------------
- ``unitree_sdk2_py``  (see README.md for installation)
- ``matplotlib``, ``numpy``

Controls
--------
- Press ``q`` or close the window to exit.

Author: unitree_rl_lab
"""

from __future__ import annotations

import argparse
import collections
import os
import sys
import time
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
#  Graceful import of unitree_sdk2_py
# ---------------------------------------------------------------------------
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
except ImportError:
    print(
        "\n⚠  unitree_sdk2_py is not installed.\n"
        "   Install it from the SDK source:\n"
        "     cd <unitree_sdk2_python>\n"
        "     pip install -e .\n"
        "   See README.md § 'Deploy' for details.\n",
        file=sys.stderr,
    )
    sys.exit(1)

import matplotlib
# Auto-select a GUI backend: prefer TkAgg (tkinter — bundled with CPython)
import importlib.util
if importlib.util.find_spec("tkinter") is not None:
    matplotlib.use("TkAgg")
elif importlib.util.find_spec("PyQt5") is not None:
    matplotlib.use("Qt5Agg")
elif importlib.util.find_spec("PySide2") is not None:
    matplotlib.use("Qt5Agg")
else:
    matplotlib.use("WebAgg")  # last resort: opens in browser
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
from matplotlib.patches import FancyBboxPatch

# ---------------------------------------------------------------------------
#  G1 Joint Index Map  (from g1_low_level_example.py)
# ---------------------------------------------------------------------------

G1_NUM_MOTOR = 29

class J:
    """G1 29-dof joint indices."""
    # Left leg
    LeftHipPitch     = 0
    LeftHipRoll      = 1
    LeftHipYaw       = 2
    LeftKnee         = 3
    LeftAnklePitch   = 4
    LeftAnkleRoll    = 5
    # Right leg
    RightHipPitch    = 6
    RightHipRoll     = 7
    RightHipYaw      = 8
    RightKnee        = 9
    RightAnklePitch  = 10
    RightAnkleRoll   = 11
    # Waist
    WaistYaw         = 12
    WaistRoll        = 13  # locked on G1
    WaistPitch       = 14  # locked on G1
    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll  = 16
    LeftShoulderYaw   = 17
    LeftElbow         = 18
    LeftWristRoll     = 19
    LeftWristPitch    = 20
    LeftWristYaw      = 21
    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll  = 23
    RightShoulderYaw   = 24
    RightElbow         = 25
    RightWristRoll     = 26
    RightWristPitch    = 27
    RightWristYaw      = 28

# ---------------------------------------------------------------------------
#  Joint group definitions  (name, short_label, indices, color)
# ---------------------------------------------------------------------------
JOINT_GROUPS = [
    {
        "name": "Left Leg",
        "labels": ["HipPitch", "HipRoll", "HipYaw", "Knee", "AnklePitch", "AnkleRoll"],
        "indices": [0, 1, 2, 3, 4, 5],
        "color": "#4ECDC4",  # teal
    },
    {
        "name": "Right Leg",
        "labels": ["HipPitch", "HipRoll", "HipYaw", "Knee", "AnklePitch", "AnkleRoll"],
        "indices": [6, 7, 8, 9, 10, 11],
        "color": "#FF6B6B",  # coral
    },
    {
        "name": "Waist",
        "labels": ["Yaw", "Roll †", "Pitch †"],
        "indices": [12, 13, 14],
        "color": "#FFE66D",  # gold
    },
    {
        "name": "Left Arm",
        "labels": ["ShldPitch", "ShldRoll", "ShldYaw", "Elbow", "WristRoll", "WristPitch", "WristYaw"],
        "indices": [15, 16, 17, 18, 19, 20, 21],
        "color": "#95E1D3",  # mint
    },
    {
        "name": "Right Arm",
        "labels": ["ShldPitch", "ShldRoll", "ShldYaw", "Elbow", "WristRoll", "WristPitch", "WristYaw"],
        "indices": [22, 23, 24, 25, 26, 27, 28],
        "color": "#F38181",  # salmon
    },
]

# Flatten: one entry per joint 0..28
JOINT_FLAT = []
for grp in JOINT_GROUPS:
    for i, idx in enumerate(grp["indices"]):
        JOINT_FLAT.append({
            "index": idx,
            "label": grp["labels"][i],
            "group": grp["name"],
            "color": grp["color"],
        })
JOINT_FLAT.sort(key=lambda x: x["index"])

# ---------------------------------------------------------------------------
#  Ring buffer for history plots
# ---------------------------------------------------------------------------
HISTORY_LENGTH = 200  # frames (~20 s at 10 Hz display)


@dataclass
class RobotState:
    """Snapshot of the latest robot state (populated by DDS callback)."""
    q: np.ndarray = field(default_factory=lambda: np.zeros(G1_NUM_MOTOR))
    dq: np.ndarray = field(default_factory=lambda: np.zeros(G1_NUM_MOTOR))
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tick: int = 0
    mode_pr: int = 0
    mode_machine: int = 0
    timestamp: float = 0.0

    def update(self, msg: LowState_):
        for i in range(G1_NUM_MOTOR):
            self.q[i] = msg.motor_state[i].q
            self.dq[i] = msg.motor_state[i].dq
        self.rpy[:] = msg.imu_state.rpy
        self.gyro[:] = msg.imu_state.gyroscope
        self.accel[:] = msg.imu_state.accelerometer
        self.tick = msg.tick
        self.mode_pr = msg.mode_pr
        self.mode_machine = msg.mode_machine
        self.timestamp = time.monotonic()


# ---------------------------------------------------------------------------
#  Main Visualizer
# ---------------------------------------------------------------------------

class G1StateVisualizer:
    """Real-time matplotlib dashboard for G1 robot state."""

    def __init__(self, network: str = "", domain: int = 0, scale: float = 1.0):
        self.network = network
        self.domain = domain
        self.scale = float(scale)
        self.state = RobotState()
        self.running = True

        # Ring buffers for history traces
        self.history_q: dict[int, collections.deque] = {
            i: collections.deque(maxlen=HISTORY_LENGTH) for i in range(G1_NUM_MOTOR)
        }
        self.history_rpy: list[collections.deque] = [
            collections.deque(maxlen=HISTORY_LENGTH) for _ in range(3)
        ]
        self.history_time: collections.deque = collections.deque(
            maxlen=HISTORY_LENGTH
        )

        # Setup DDS
        self._setup_dds()

        # Build GUI
        self._setup_gui()

    def _s(self, base: float) -> float:
        """Scale a size value (fontsize, linewidth, etc.) by the user's scale factor."""
        return base * self.scale

    # -----------------------------------------------------------------------
    #  DDS
    # -----------------------------------------------------------------------

    def _setup_dds(self):
        kwargs = {}
        if self.network:
            kwargs["networkInterface"] = self.network
        ChannelFactoryInitialize(self.domain, **kwargs)

        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_lowstate, 10)

        print(f"✓ Subscribed to rt/lowstate  (domain={self.domain}, "
              f"iface={self.network or 'auto'})")
        print("  Waiting for data...")

        # Wait until the first message arrives
        timeout = 5.0
        t0 = time.monotonic()
        while self.state.tick == 0:
            time.sleep(0.05)
            if time.monotonic() - t0 > timeout:
                print("⚠  No data received within 5 s.  Is the robot running?")
                break

    def _on_lowstate(self, msg: LowState_):
        self.state.update(msg)

    # -----------------------------------------------------------------------
    #  GUI
    # -----------------------------------------------------------------------

    def _setup_gui(self):
        plt.style.use("dark_background")
        # Scale default font sizes so explicit overrides are the exception
        matplotlib.rcParams.update({
            "font.size":           self._s(10),
            "axes.titlesize":      self._s(12),
            "axes.labelsize":      self._s(9),
            "xtick.labelsize":     self._s(8),
            "ytick.labelsize":     self._s(8),
            "legend.fontsize":     self._s(8),
            "figure.dpi":          100 * self.scale,
        })
        self.fig = plt.figure("G1-29dof State Monitor", figsize=(16, 9))
        self.fig.patch.set_facecolor("#1a1a2e")

        gs = GridSpec(3, 3, figure=self.fig,
                      width_ratios=[1.0, 1.8, 1.8],
                      height_ratios=[1.2, 1.0, 0.18],
                      hspace=0.45, wspace=0.35,
                      left=0.05, right=0.97, top=0.93, bottom=0.06)

        # ---- Row 0: IMU overview + Joint q ----
        self.ax_imu = self.fig.add_subplot(gs[0, 0])
        self.ax_q   = self.fig.add_subplot(gs[0, 1:])

        # ---- Row 1: IMU traces + Joint dq ----
        self.ax_imu_hist = self.fig.add_subplot(gs[1, 0])
        self.ax_dq        = self.fig.add_subplot(gs[1, 1:])

        # ---- Row 2: Status bar ----
        self.ax_status = self.fig.add_subplot(gs[2, :])
        self.ax_status.axis("off")

        self._init_imu_panel()
        self._init_joint_q_panel()
        self._init_imu_history_panel()
        self._init_joint_dq_panel()

        self.fig.canvas.manager.set_window_title("G1-29dof State Monitor")

    # -------------------------------------------------------------------
    #  IMU Panel  (attitude + text readout)
    # -------------------------------------------------------------------

    def _init_imu_panel(self):
        ax = self.ax_imu
        ax.set_title("IMU Attitude", fontsize=self._s(12), fontweight="bold",
                     color="#e0e0e0", pad=self._s(10))
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect("equal")
        ax.axis("off")

        # Draw a circular gauge background
        theta = np.linspace(0, 2 * np.pi, 200)
        for r in [1.0, 1.3]:
            ax.plot(r * np.cos(theta), r * np.sin(theta,
                    ), color="#333355", lw=self._s(1.5), zorder=0)

        # Tick marks
        for deg in range(0, 360, 30):
            rad = np.radians(deg)
            x0, y0 = 1.15 * np.cos(rad), 1.15 * np.sin(rad)
            x1, y1 = 1.30 * np.cos(rad), 1.30 * np.sin(rad)
            ax.plot([x0, x1], [y0, y1], color="#555577", lw=self._s(1), zorder=0)

        # Horizon line  (roll indicator)
        self.horizon_line, = ax.plot([], [], color="#FF6B6B", lw=self._s(2.5),
                                     alpha=0.7, zorder=2)
        # Pitch indicator
        self.pitch_arrow, = ax.plot([], [], color="#4ECDC4", lw=self._s(3), zorder=3)
        # Yaw text
        self.yaw_text = ax.text(0, -1.45, "", ha="center", va="top",
                                fontsize=self._s(9), color="#aaaacc",
                                fontfamily="monospace")

        # Text readout (right side numbers)
        self.imu_text = ax.text(
            0.95, 0.95, "",
            transform=ax.transAxes, ha="right", va="top",
            fontsize=self._s(8), color="#ccccdd", fontfamily="monospace",
            linespacing=1.8,
        )

    # -------------------------------------------------------------------
    #  Joint Position Panel  (horizontal bar chart)
    # -------------------------------------------------------------------

    def _init_joint_q_panel(self):
        ax = self.ax_q
        ax.set_title("Joint Positions  q [rad]", fontsize=self._s(12),
                     fontweight="bold", color="#e0e0e0", pad=self._s(10))

        n = G1_NUM_MOTOR
        y_pos = range(n)
        labels = [j["label"] for j in JOINT_FLAT]
        colors = [j["color"] for j in JOINT_FLAT]

        self.q_bars = ax.barh(y_pos, np.zeros(n), height=self._s(0.7),
                              color=colors, alpha=0.85, edgecolor="#222244",
                              linewidth=self._s(0.5))

        ax.set_yticks(y_pos)
        ax.set_yticklabels(labels, fontsize=self._s(7), fontfamily="monospace")
        ax.set_xlim(-3.5, 3.5)
        ax.axvline(0, color="#444466", lw=self._s(0.8))
        ax.invert_yaxis()

        # Add group separator lines
        sep_positions = [6, 12, 15, 22]
        for sp in sep_positions:
            ax.axhline(sp - 0.5, color="#444466", lw=self._s(1), ls="--", alpha=0.5)

        # Group labels on right y-axis
        group_labels = [(3, "L Leg"), (9, "R Leg"), (13.5, "Waist"),
                        (18.5, "L Arm"), (25.5, "R Arm")]
        ax_right = ax.twinx()
        ax_right.set_ylim(ax.get_ylim())
        ax_right.set_yticks([pos for pos, _ in group_labels])
        ax_right.set_yticklabels([lbl for _, lbl in group_labels],
                                 fontsize=self._s(7), fontfamily="monospace",
                                 color="#888899", rotation=90, va="center")
        ax_right.tick_params(length=0)

        ax.tick_params(axis="x", labelsize=self._s(7), colors="#888899")

    # -------------------------------------------------------------------
    #  IMU History Panel
    # -------------------------------------------------------------------

    def _init_imu_history_panel(self):
        ax = self.ax_imu_hist
        ax.set_title("IMU Roll / Pitch / Yaw", fontsize=self._s(10),
                     fontweight="bold", color="#e0e0e0", pad=self._s(8))
        ax.set_ylabel("[rad]", fontsize=self._s(8), color="#888899")
        ax.set_xlabel("time [s]", fontsize=self._s(8), color="#888899")

        colors = ["#FF6B6B", "#4ECDC4", "#FFE66D"]  # R, P, Y
        names  = ["Roll", "Pitch", "Yaw"]
        self.imu_lines = []
        for i in range(3):
            (line,) = ax.plot([], [], color=colors[i], lw=self._s(1.5),
                              label=names[i], alpha=0.9)
            self.imu_lines.append(line)
        ax.legend(fontsize=self._s(7), loc="upper right",
                  framealpha=0.3, edgecolor="#444466")
        ax.set_ylim(-3.5, 3.5)
        ax.axhline(0, color="#444466", lw=self._s(0.5))
        ax.tick_params(labelsize=self._s(7), colors="#888899")
        ax.set_facecolor("#12122a")

    # -------------------------------------------------------------------
    #  Joint Velocity Panel  (horizontal bar chart)
    # -------------------------------------------------------------------

    def _init_joint_dq_panel(self):
        ax = self.ax_dq
        ax.set_title("Joint Velocities  dq [rad/s]", fontsize=self._s(12),
                     fontweight="bold", color="#e0e0e0", pad=self._s(10))

        n = G1_NUM_MOTOR
        y_pos = range(n)
        labels = [j["label"] for j in JOINT_FLAT]
        colors = [j["color"] for j in JOINT_FLAT]

        self.dq_bars = ax.barh(y_pos, np.zeros(n), height=self._s(0.7),
                               color=colors, alpha=0.85, edgecolor="#222244",
                               linewidth=self._s(0.5))

        ax.set_yticks(y_pos)
        ax.set_yticklabels(labels, fontsize=self._s(7), fontfamily="monospace")
        ax.set_xlim(-12, 12)
        ax.axvline(0, color="#444466", lw=self._s(0.8))
        ax.invert_yaxis()

        # Group separators
        for sp in [6, 12, 15, 22]:
            ax.axhline(sp - 0.5, color="#444466", lw=self._s(1), ls="--", alpha=0.5)

        # Group labels
        group_labels = [(3, "L Leg"), (9, "R Leg"), (13.5, "Waist"),
                        (18.5, "L Arm"), (25.5, "R Arm")]
        ax_right = ax.twinx()
        ax_right.set_ylim(ax.get_ylim())
        ax_right.set_yticks([pos for pos, _ in group_labels])
        ax_right.set_yticklabels([lbl for _, lbl in group_labels],
                                 fontsize=self._s(7), fontfamily="monospace",
                                 color="#888899", rotation=90, va="center")
        ax_right.tick_params(length=0)
        ax.tick_params(axis="x", labelsize=self._s(7), colors="#888899")

    # -----------------------------------------------------------------------
    #  Update loop
    # -----------------------------------------------------------------------

    def _update(self, frame: int):
        """Called by FuncAnimation at ~10 Hz."""
        q = self.state.q
        dq = self.state.dq
        rpy = self.state.rpy
        now = time.monotonic()

        # ---- push history ----
        t_hist = now
        self.history_time.append(t_hist)
        for i in range(G1_NUM_MOTOR):
            self.history_q[i].append(q[i])
        for i in range(3):
            self.history_rpy[i].append(rpy[i])

        # ---- joint q bars ----
        for i, bar in enumerate(self.q_bars):
            bar.set_width(q[i])
            # Highlight out-of-range joints
            if abs(q[i]) > 3.0:
                bar.set_edgecolor("#FF4444")
                bar.set_linewidth(self._s(1.2))
            else:
                bar.set_edgecolor("#222244")
                bar.set_linewidth(self._s(0.5))

        # ---- joint dq bars ----
        for i, bar in enumerate(self.dq_bars):
            bar.set_width(dq[i])
            if abs(dq[i]) > 10.0:
                bar.set_edgecolor("#FF4444")
                bar.set_linewidth(self._s(1.2))
            else:
                bar.set_edgecolor("#222244")
                bar.set_linewidth(self._s(0.5))

        # ---- IMU panel ----
        self._update_imu_panel(rpy)

        # ---- IMU history ----
        times = list(self.history_time)
        # Normalise to relative seconds
        if len(times) > 1:
            t0 = times[0]
            rel_times = [t - t0 for t in times]
            for i, line in enumerate(self.imu_lines):
                data = list(self.history_rpy[i])
                line.set_data(rel_times[-len(data):], data[-len(rel_times):])
                if rel_times:
                    x_pad = max(1.0, rel_times[-1] * 0.05)
                    self.ax_imu_hist.set_xlim(
                        rel_times[0] - x_pad, rel_times[-1] + x_pad)
                else:
                    self.ax_imu_hist.set_xlim(-1, 1)

        # ---- status bar ----
        self._update_status()

        return (
            *self.q_bars, *self.dq_bars, *self.imu_lines,
            self.horizon_line, self.pitch_arrow, self.yaw_text, self.imu_text,
        )

    def _update_imu_panel(self, rpy: np.ndarray):
        """Update the IMU attitude display."""
        roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])

        # Horizon line (roll)
        L = 1.25
        cos_r, sin_r = np.cos(roll), np.sin(roll)
        self.horizon_line.set_data(
            [-L * cos_r, L * cos_r],
            [-L * sin_r, L * sin_r],
        )

        # Pitch arrow  (radial, length ∝ sin(pitch))
        arrow_len = np.clip(np.sin(pitch) * 1.0, -1.0, 1.0)
        self.pitch_arrow.set_data([0, 0], [0, arrow_len])

        # Yaw text (compass-style)
        yaw_deg = np.degrees(yaw) % 360
        directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        idx = int((yaw_deg + 22.5) // 45) % 8
        self.yaw_text.set_text(f"Yaw: {yaw_deg:6.1f}°  ({directions[idx]})")

        # Text readout
        self.imu_text.set_text(
            f"Roll:   {np.degrees(roll):+7.2f}°\n"
            f"Pitch:  {np.degrees(pitch):+7.2f}°\n"
            f"Yaw:    {np.degrees(yaw):+7.2f}°\n"
            f"\n"
            f"Gyro x: {self.state.gyro[0]:+7.3f}\n"
            f"Gyro y: {self.state.gyro[1]:+7.3f}\n"
            f"Gyro z: {self.state.gyro[2]:+7.3f}\n"
            f"\n"
            f"Acc x:  {self.state.accel[0]:+7.3f}\n"
            f"Acc y:  {self.state.accel[1]:+7.3f}\n"
            f"Acc z:  {self.state.accel[2]:+7.3f}"
        )

    def _update_status(self):
        """Render the status bar."""
        ax = self.ax_status
        ax.clear()
        ax.axis("off")

        mode_names = {0: "PR", 1: "AB"}
        mode_pr_str = mode_names.get(self.state.mode_pr, str(self.state.mode_pr))

        status_line = (
            f"Tick: {self.state.tick:<10d}"
            f"  |  Mode PR: {mode_pr_str:<3s}"
            f"  |  Mode Machine: {self.state.mode_machine}"
            f"  |  IMU  R={np.degrees(self.state.rpy[0]):+6.1f}°  "
            f"P={np.degrees(self.state.rpy[1]):+6.1f}°  "
            f"Y={np.degrees(self.state.rpy[2]):+6.1f}°"
            f"  |  FPS: {self._fps():5.1f}"
        )
        ax.text(0.5, 0.5, status_line, transform=ax.transAxes,
                ha="center", va="center", fontsize=self._s(8.5),
                color="#aabbcc", fontfamily="monospace")

        # Background box
        ax.add_patch(
            plt.Rectangle((0, 0), 1, 1, transform=ax.transAxes,
                          facecolor="#0f0f23", edgecolor="#333355",
                          linewidth=self._s(1), zorder=-1)
        )

        # Track FPS timing
        now = time.monotonic()
        if not hasattr(self, "_last_frame_time"):
            self._last_frame_time = now
            self._fps_history = collections.deque(maxlen=20)
        dt = now - self._last_frame_time
        if dt > 0:
            self._fps_history.append(1.0 / dt)
        self._last_frame_time = now

    def _fps(self) -> float:
        if hasattr(self, "_fps_history") and self._fps_history:
            return np.mean(self._fps_history)
        return 0.0

    # -----------------------------------------------------------------------
    #  Run
    # -----------------------------------------------------------------------

    def run(self):
        self.ani = animation.FuncAnimation(
            self.fig, self._update,
            interval=80,  # ~12 Hz refresh
            blit=False,
            cache_frame_data=False,
        )
        plt.show()


# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="G1-29dof Robot State Visualizer"
    )
    parser.add_argument(
        "--network", "-n",
        type=str,
        default="",
        help="DDS network interface (e.g. enp5s0, lo).  Auto-detect if omitted.",
    )
    parser.add_argument(
        "--domain", "-d",
        type=int,
        default=0,
        help="DDS domain ID (default: 0)",
    )
    parser.add_argument(
        "--scale", "-s",
        type=float,
        default=1.0,
        help="GUI scale factor for HiDPI screens (default: 1.0, try 1.5 or 2.0)",
    )
    args = parser.parse_args()

    vis = G1StateVisualizer(network=args.network, domain=args.domain,
                            scale=args.scale)
    vis.run()


if __name__ == "__main__":
    main()
