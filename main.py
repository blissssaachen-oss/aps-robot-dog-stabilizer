from __future__ import annotations

import os
import signal
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from controllers import LADRCController, PIDController
from gait import TrotGait
from ik import IKSolver, JOINTS_OF_INTEREST
from imu_utils import ComplementaryFilterRP, ExponentialSmoother, quat_to_roll_pitch_yaw
from logging_utils import CsvLogger


DEFAULT_IMU_TOPICS: Sequence[str] = (
    "/imu",
    "/imu/data",
    "/imu_sensor_broadcaster/imu",
    "/imu_sensor/imu",
)


@dataclass
class APSConfig:
    control_rate_hz: float = 200.0
    gait_rate_hz: float = 50.0

    # Controller selection: "pid" or "ladrc"
    controller_type: str = "pid"

    # PID gains (per axis)
    pid_kp: float = 2.0
    pid_ki: float = 0.0
    pid_kd: float = 0.2

    # LADRC gains (per axis)
    ladrc_b0: float = 1.0
    ladrc_w0: float = 20.0
    ladrc_kp: float = 6.0
    ladrc_kd: float = 1.0

    # Mapping controller outputs into ΔZ per-leg
    dz_gain_roll: float = 0.02   # meters per rad of roll correction
    dz_gain_pitch: float = 0.02  # meters per rad of pitch correction
    dz_limit: float = 0.05       # meters

    # Optional smoothing on IMU angles
    smooth_alpha: float = 0.2

    # Complementary filter (accel+gyro) for roll/pitch. If False, use msg.orientation quaternion.
    use_complementary_filter: bool = True
    complementary_alpha: float = 0.98

    # Mode: "stand" or "trot"
    mode: str = "trot"

    # Logging
    log_dir: str = os.path.join(os.path.dirname(__file__), "logs")


class APSNode(Node):
    def __init__(self, cfg: APSConfig):
        super().__init__("aps_node")
        self.cfg = cfg

        self._lock = threading.Lock()
        self._roll: Optional[float] = None
        self._pitch: Optional[float] = None
        self._yaw: Optional[float] = None
        self._last_imu_time: Optional[float] = None

        self._joint_positions: Optional[np.ndarray] = None
        self._last_joint_time: Optional[float] = None

        self._imu_topic = os.environ.get("APS_IMU_TOPIC", "").strip() or None

        self._roll_s = ExponentialSmoother(alpha=cfg.smooth_alpha)
        self._pitch_s = ExponentialSmoother(alpha=cfg.smooth_alpha)

        self._comp = ComplementaryFilterRP(
            dt=1.0 / cfg.control_rate_hz,
            alpha=cfg.complementary_alpha,
        )

        self._joint_sub = self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self._imu_sub = None
        self._try_create_imu_subscription()

        self._cmd_pub = self.create_publisher(Float64MultiArray, "/forward_command_controller/commands", 10)

        self._gait = TrotGait()
        self._ik = IKSolver(max_iter=100)

        dt = 1.0 / self.cfg.control_rate_hz
        self._pid_roll = PIDController(
            kp=cfg.pid_kp, ki=cfg.pid_ki, kd=cfg.pid_kd, dt=dt, integrator_limit=0.5, output_limit=2.0
        )
        self._pid_pitch = PIDController(
            kp=cfg.pid_kp, ki=cfg.pid_ki, kd=cfg.pid_kd, dt=dt, integrator_limit=0.5, output_limit=2.0
        )
        self._ladrc_roll = LADRCController(
            dt=dt,
            b0=cfg.ladrc_b0,
            w0=cfg.ladrc_w0,
            kp=cfg.ladrc_kp,
            kd=cfg.ladrc_kd,
            u_limit=2.0,
        )
        self._ladrc_pitch = LADRCController(
            dt=dt,
            b0=cfg.ladrc_b0,
            w0=cfg.ladrc_w0,
            kp=cfg.ladrc_kp,
            kd=cfg.ladrc_kd,
            u_limit=2.0,
        )

        self._phase = 0.0
        self._phase_rate = 1.0  # cycles/sec (used only if trotting)

        self._target_joint_guess = np.zeros(12, dtype=float)

        self._logger = CsvLogger(out_dir=self.cfg.log_dir, filename_prefix=f"aps_{cfg.controller_type}_{cfg.mode}", flush_every_n=1)
        self._log_path = self._logger.start(
            [
                "t",
                "roll",
                "pitch",
                "yaw",
                "u_roll",
                "u_pitch",
                "dz_rf",
                "dz_lf",
                "dz_rb",
                "dz_lb",
            ]
        )
        self.get_logger().info(f"Logging to {self._log_path}")

        self.create_timer(1.0 / self.cfg.control_rate_hz, self._control_tick)

    # ---------------- subscriptions ----------------
    def _try_create_imu_subscription(self) -> None:
        if self._imu_sub is not None:
            return

        # If user forced a topic, try that first.
        candidates: List[str] = []
        if self._imu_topic:
            candidates.append(self._imu_topic)
        candidates.extend([t for t in DEFAULT_IMU_TOPICS if t != self._imu_topic])

        # Best-effort: create a subscription and see if messages arrive (we log warnings if not).
        # Note: rclpy doesn't provide a cheap, portable "topic exists" check without graph queries;
        # this approach keeps the node simple.
        topic = candidates[0]
        self._imu_topic = topic
        self._imu_sub = self.create_subscription(Imu, topic, self._on_imu, 10)
        self.get_logger().info(f"Subscribed IMU on {topic}. Set APS_IMU_TOPIC to override.")

    def _on_imu(self, msg: Imu) -> None:
        if self.cfg.use_complementary_filter:
            a = msg.linear_acceleration
            g = msg.angular_velocity
            # gyro x,y correspond to roll/pitch rates in body frame for small angles
            roll, pitch = self._comp.update(ax=a.x, ay=a.y, az=a.z, gx=g.x, gy=g.y)
            yaw = 0.0
        else:
            q = msg.orientation
            roll, pitch, yaw = quat_to_roll_pitch_yaw(q.x, q.y, q.z, q.w)

        roll = self._roll_s.update(roll)
        pitch = self._pitch_s.update(pitch)
        with self._lock:
            self._roll = float(roll)
            self._pitch = float(pitch)
            self._yaw = float(yaw)
            self._last_imu_time = time.time()

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            pos = np.array([msg.position[msg.name.index(j)] for j in JOINTS_OF_INTEREST], dtype=float)
        except ValueError:
            return
        with self._lock:
            self._joint_positions = pos
            self._last_joint_time = time.time()

    # ---------------- control ----------------
    def _compute_leg_dz(self, u_roll: float, u_pitch: float) -> np.ndarray:
        """
        Map roll/pitch control outputs into ΔZ per leg in order [RF, LF, RB, LB].

        Convention (can be tuned by flipping dz_gain_* signs):
        - Positive u_roll raises left legs, lowers right legs.
        - Positive u_pitch raises rear legs, lowers front legs.
        """
        dr = float(np.clip(self.cfg.dz_gain_roll * u_roll, -self.cfg.dz_limit, self.cfg.dz_limit))
        dp = float(np.clip(self.cfg.dz_gain_pitch * u_pitch, -self.cfg.dz_limit, self.cfg.dz_limit))

        dz_rf = -dr - dp
        dz_lf = +dr - dp
        dz_rb = -dr + dp
        dz_lb = +dr + dp
        return np.array([dz_rf, dz_lf, dz_rb, dz_lb], dtype=float)

    def _control_tick(self) -> None:
        now = time.time()
        with self._lock:
            roll = self._roll
            pitch = self._pitch
            yaw = self._yaw
            joint_pos = self._joint_positions
            last_imu = self._last_imu_time

        if roll is None or pitch is None or yaw is None or last_imu is None:
            return
        if joint_pos is None:
            return

        # If we never received IMU on this topic, try rotating through candidates after a timeout.
        if now - last_imu > 1.0 and self._imu_sub is not None:
            # cycle to the next default topic (best effort)
            candidates = [self._imu_topic] + [t for t in DEFAULT_IMU_TOPICS if t != self._imu_topic]
            if len(candidates) > 1:
                next_topic = candidates[1]
                self.get_logger().warning(f"No IMU data for 1s on {self._imu_topic}. Trying {next_topic}.")
                self.destroy_subscription(self._imu_sub)
                self._imu_sub = self.create_subscription(Imu, next_topic, self._on_imu, 10)
                self._imu_topic = next_topic
            return

        # Setpoints: level body.
        e_roll = 0.0 - roll
        e_pitch = 0.0 - pitch

        if self.cfg.controller_type.lower() == "ladrc":
            u_roll = self._ladrc_roll.update(y=roll, r=0.0)
            u_pitch = self._ladrc_pitch.update(y=pitch, r=0.0)
        else:
            u_roll = self._pid_roll.update(e_roll)
            u_pitch = self._pid_pitch.update(e_pitch)

        dz = self._compute_leg_dz(u_roll=u_roll, u_pitch=u_pitch)

        # Nominal end-effector targets.
        if self.cfg.mode.lower() == "stand":
            # One snapshot of the gait at phase=0 corresponds to a stable standing footprint.
            ee_nominal = self._gait.sample_all_legs(0.0)
        else:
            ee_nominal = self._gait.sample_all_legs(self._phase)
            self._phase = (self._phase + (self._phase_rate / self.cfg.control_rate_hz)) % 1.0

        ee_cmd = ee_nominal.copy()
        # Add ΔZ offsets per leg
        for leg in range(4):
            ee_cmd[3 * leg + 2] += dz[leg]

        try:
            q_cmd = self._ik.inverse_kinematics_all_legs(ee_cmd, initial_guess_all=self._target_joint_guess)
        except Exception as ex:
            self.get_logger().error(f"IK failed: {ex}")
            return

        self._target_joint_guess = q_cmd

        msg = Float64MultiArray()
        msg.data = q_cmd.tolist()
        self._cmd_pub.publish(msg)

        self._logger.log(
            {
                "t": now,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "u_roll": float(u_roll),
                "u_pitch": float(u_pitch),
                "dz_rf": float(dz[0]),
                "dz_lf": float(dz[1]),
                "dz_rb": float(dz[2]),
                "dz_lb": float(dz[3]),
            }
        )

    def destroy_node(self):
        try:
            self._logger.close()
        finally:
            super().destroy_node()


def load_config() -> APSConfig:
    cfg = APSConfig()
    cfg.controller_type = os.environ.get("APS_CONTROLLER", cfg.controller_type).strip().lower()
    cfg.mode = os.environ.get("APS_MODE", cfg.mode).strip().lower()
    cfg.log_dir = os.environ.get("APS_LOG_DIR", cfg.log_dir).strip() or cfg.log_dir
    cfg.use_complementary_filter = os.environ.get("APS_USE_COMP", "1").strip() not in ("0", "false", "False")
    try:
        cfg.control_rate_hz = float(os.environ.get("APS_CONTROL_RATE_HZ", cfg.control_rate_hz))
        cfg.gait_rate_hz = float(os.environ.get("APS_GAIT_RATE_HZ", cfg.gait_rate_hz))
    except ValueError:
        pass
    return cfg


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    cfg = load_config()
    node = APSNode(cfg)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())

    try:
        while rclpy.ok() and not stop_requested.is_set():
            time.sleep(0.1)
    finally:
        # Best-effort safe stop: publish a few times the last known command (or zeros if none).
        for _ in range(3):
            node._cmd_pub.publish(Float64MultiArray(data=[0.0] * 12))
            time.sleep(0.05)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
