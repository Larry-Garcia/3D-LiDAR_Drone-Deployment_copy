#!/usr/bin/env python3
"""
Single-file 2D pan tracking stack for easier integration.

This file consolidates logic from:
- SOURCE: servo_control/scripts/object_motion_detector_2d.py
- SOURCE: servo_control/scripts/object_pan_tracker_2d.py
- SOURCE: servo_control/scripts/servo_motor_node.py
- SOURCE (optional sweep behavior): servo_control/src/scan_sequence_node.cpp

CMAKE UPDATE BLOCK (added in servo_control/CMakeLists.txt):
install(PROGRAMS
  scripts/2D_Pan_tracker_single.py
  DESTINATION lib/${PROJECT_NAME}
)
"""

import json
import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray
from visualization_msgs.msg import Marker

try:
    import serial
except Exception:
    serial = None


class PanTracker2DSingleNode(Node):
    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        if v < lo:
            return lo
        if v > hi:
            return hi
        return v

    def __init__(self) -> None:
        super().__init__("pan_tracker_2d_single")

        # ---------------------------------------------------------------------
        # Block A: topics and outputs
        # SOURCE: object_motion_detector_2d.py + object_pan_tracker_2d.py
        # ---------------------------------------------------------------------
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.detect_topic = self.declare_parameter(
            "detect_topic", "/moving_object_detected").value
        self.actuate_topic = self.declare_parameter(
            "actuate_topic", "/actuation_trigger").value
        self.target_angle_topic = self.declare_parameter(
            "target_angle_topic", "/moving_object_angle_deg").value
        self.target_range_topic = self.declare_parameter(
            "target_range_topic", "/moving_object_range_m").value
        self.marker_topic = self.declare_parameter(
            "marker_topic", "/moving_object_marker").value
        self.pan_state_topic = self.declare_parameter(
            "pan_state_topic", "/tracker_pan_cmd_deg").value
        self.pan_desired_topic = self.declare_parameter(
            "pan_desired_topic", "/tracker_pan_desired_deg").value
        self.servo_topic = self.declare_parameter(
            "servo_topic", "/servo_angles").value

        # ---------------------------------------------------------------------
        # Block B: detector settings
        # SOURCE: object_motion_detector_2d.py
        # ---------------------------------------------------------------------
        self.publish_marker = self._as_bool(
            self.declare_parameter("publish_marker", True).value)
        self.marker_size_m = float(
            self.declare_parameter("marker_size_m", 0.20).value)

        self.sector_min_deg = float(
            self.declare_parameter("sector_min_deg", -180.0).value)
        self.sector_max_deg = float(
            self.declare_parameter("sector_max_deg", 180.0).value)
        if self.sector_max_deg < self.sector_min_deg:
            self.sector_min_deg, self.sector_max_deg = self.sector_max_deg, self.sector_min_deg

        self.min_range_m = float(
            self.declare_parameter("min_range_m", 0.25).value)
        self.max_range_m = float(
            self.declare_parameter("max_range_m", 8.0).value)
        self.delta_range_m = float(
            self.declare_parameter("delta_range_m", 0.02).value)
        self.min_cluster_beams = int(
            self.declare_parameter("min_cluster_beams", 1).value)
        self.min_motion_frames = int(
            self.declare_parameter("min_motion_frames", 1).value)
        self.trigger_cooldown_sec = float(
            self.declare_parameter("trigger_cooldown_sec", 0.75).value)
        self.track_with_nearest_fallback = self._as_bool(
            self.declare_parameter("track_with_nearest_fallback", False).value)
        self.max_pan_change_for_motion_deg = float(
            self.declare_parameter("max_pan_change_for_motion_deg", 0.3).value)

        # ---------------------------------------------------------------------
        # Block C: tracker settings
        # SOURCE: object_pan_tracker_2d.py + object_motion_detector_2d.py
        # ---------------------------------------------------------------------
        self.enable_pan_tracking = self._as_bool(
            self.declare_parameter("enable_pan_tracking", True).value)
        self.tracker_publish_rate_hz = float(
            self.declare_parameter("tracker_publish_rate_hz", 12.0).value)
        if self.tracker_publish_rate_hz <= 0.0:
            self.tracker_publish_rate_hz = 12.0

        self.pan_min_deg = float(self.declare_parameter("pan_min_deg", -90.0).value)
        self.pan_max_deg = float(self.declare_parameter("pan_max_deg", 90.0).value)
        if self.pan_max_deg < self.pan_min_deg:
            self.pan_min_deg, self.pan_max_deg = self.pan_max_deg, self.pan_min_deg

        self.pan_center_deg = float(self.declare_parameter("pan_center_deg", 0.0).value)
        self.start_pan_deg = float(self.declare_parameter("start_pan_deg", 0.0).value)
        self.tilt_deg = float(self.declare_parameter("tilt_deg", 0.0).value)
        self.k_p = float(self.declare_parameter("k_p", 0.45).value)
        self.max_step_deg = float(self.declare_parameter("max_step_deg", 8.0).value)
        self.deadband_deg = float(self.declare_parameter("deadband_deg", 0.8).value)
        self.target_timeout_sec = float(
            self.declare_parameter("target_timeout_sec", 1.0).value)
        self.track_require_detected = self._as_bool(
            self.declare_parameter("track_require_detected", True).value)
        self.invert_angle = self._as_bool(
            self.declare_parameter("invert_angle", False).value)

        # ---------------------------------------------------------------------
        # Block D: optional scan sequence (sweep)
        # SOURCE: scan_sequence_node.cpp behavior, Pythonized
        # ---------------------------------------------------------------------
        self.enable_scan_sequence = self._as_bool(
            self.declare_parameter("enable_scan_sequence", False).value)
        self.sequence_timer_period_ms = int(
            self.declare_parameter("sequence_timer_period_ms", 100).value)
        self.sequence_pan_min_deg = float(
            self.declare_parameter("sequence_pan_min_deg", -90.0).value)
        self.sequence_pan_max_deg = float(
            self.declare_parameter("sequence_pan_max_deg", 90.0).value)
        self.sequence_pan_increment_deg = float(
            self.declare_parameter("sequence_pan_increment_deg", 10.0).value)
        self.sequence_dwell_ticks = int(
            self.declare_parameter("sequence_dwell_ticks", 2).value)
        if self.sequence_pan_max_deg < self.sequence_pan_min_deg:
            self.sequence_pan_min_deg, self.sequence_pan_max_deg = (
                self.sequence_pan_max_deg,
                self.sequence_pan_min_deg,
            )
        self.sequence_pan_deg = self._clamp(
            0.0, self.sequence_pan_min_deg, self.sequence_pan_max_deg)
        self.sequence_pan_dir = 1
        self.sequence_dwell = 0

        # ---------------------------------------------------------------------
        # Block E: servo serial output
        # SOURCE: servo_motor_node.py
        # ---------------------------------------------------------------------
        self.enable_serial_output = self._as_bool(
            self.declare_parameter("enable_serial_output", True).value)
        self.serial_port = self.declare_parameter("serial_port", "/dev/ttyTHS1").value
        self.serial_baud = int(self.declare_parameter("serial_baud", 115200).value)
        self.serial_spd = int(self.declare_parameter("serial_spd", 0).value)
        self.serial_acc = int(self.declare_parameter("serial_acc", 0).value)
        self.serial_conn = None
        if self.enable_serial_output:
            self._open_serial()

        # ---------------------------------------------------------------------
        # Block F: debug + state
        # ---------------------------------------------------------------------
        self.debug_enabled = self._as_bool(
            self.declare_parameter("debug_enabled", True).value)
        self.debug_every_n = int(self.declare_parameter("debug_every_n", 2).value)
        if self.debug_every_n < 1:
            self.debug_every_n = 2
        if self.min_cluster_beams < 1:
            self.min_cluster_beams = 1
        if self.min_motion_frames < 1:
            self.min_motion_frames = 1

        self.prev_ranges: Optional[List[float]] = None
        self.motion_streak = 0
        self.last_detect_state = False
        self.last_trigger_time = self.get_clock().now()
        self.last_target_angle_deg: Optional[float] = None
        self.last_target_time = self.get_clock().now()
        self.pan_cmd_deg = self._clamp(self.start_pan_deg, self.pan_min_deg, self.pan_max_deg)
        self.pan_desired_deg = self.pan_cmd_deg
        self.prev_scan_pan_deg: Optional[float] = None
        self.scan_count = 0
        self.target_source = "none"

        # ---------------------------------------------------------------------
        # ROS I/O wiring
        # ---------------------------------------------------------------------
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.detect_pub = self.create_publisher(Bool, self.detect_topic, 10)
        self.actuate_pub = self.create_publisher(Bool, self.actuate_topic, 10)
        self.angle_pub = self.create_publisher(Float32, self.target_angle_topic, 10)
        self.range_pub = self.create_publisher(Float32, self.target_range_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.pan_cmd_pub = self.create_publisher(Float32, self.pan_state_topic, 10)
        self.pan_desired_pub = self.create_publisher(Float32, self.pan_desired_topic, 10)
        self.servo_pub = self.create_publisher(Float32MultiArray, self.servo_topic, 10)

        self.tracker_timer = self.create_timer(
            1.0 / self.tracker_publish_rate_hz, self.tracker_timer_cb)
        if self.enable_scan_sequence:
            self.sequence_timer = self.create_timer(
                max(0.02, self.sequence_timer_period_ms / 1000.0), self.sequence_timer_cb)
        else:
            self.sequence_timer = None

        self.get_logger().info(
            f"2D single tracker started scan={self.scan_topic} servo={self.servo_topic} "
            f"sector=[{self.sector_min_deg:.1f},{self.sector_max_deg:.1f}] "
            f"delta={self.delta_range_m:.2f} cluster={self.min_cluster_beams} "
            f"frames={self.min_motion_frames} fallback={'on' if self.track_with_nearest_fallback else 'off'} "
            f"tracking={'on' if self.enable_pan_tracking else 'off'} "
            f"scan_sequence={'on' if self.enable_scan_sequence else 'off'}"
        )

    # -------------------------------------------------------------------------
    # Serial output helpers
    # SOURCE: servo_motor_node.py
    # -------------------------------------------------------------------------
    def _open_serial(self) -> None:
        if serial is None:
            self.get_logger().warn("pyserial unavailable; serial output disabled")
            self.enable_serial_output = False
            return
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.serial_baud, timeout=0.1)
            self.get_logger().info(f"Opened serial {self.serial_port} @ {self.serial_baud}")
        except Exception as ex:
            self.serial_conn = None
            self.get_logger().error(f"Serial open failed: {ex}")

    def _send_serial(self, pan_deg: float, tilt_deg: float) -> None:
        if not self.enable_serial_output:
            return
        if self.serial_conn is None:
            self._open_serial()
            if self.serial_conn is None:
                return
        cmd = {
            "T": 133,
            "X": float(pan_deg),
            "Y": float(tilt_deg),
            "SPD": int(self.serial_spd),
            "ACC": int(self.serial_acc),
        }
        try:
            self.serial_conn.write((json.dumps(cmd) + "\n").encode("utf-8"))
            self.serial_conn.flush()
        except Exception as ex:
            self.get_logger().error(f"Serial write failed: {ex}")
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

    # -------------------------------------------------------------------------
    # Detection helpers
    # SOURCE: object_motion_detector_2d.py
    # -------------------------------------------------------------------------
    def _beam_angle_deg(self, idx: int, angle_min: float, angle_inc: float) -> float:
        return (angle_min + idx * angle_inc) * 180.0 / math.pi

    def _is_valid_range(self, r: float) -> bool:
        return math.isfinite(r) and self.min_range_m <= r <= self.max_range_m

    def _find_moving_cluster(
        self, curr: List[float], prev: List[float], angle_min: float, angle_inc: float
    ) -> Tuple[bool, int, Optional[float], Optional[float]]:
        run = 0
        max_run = 0
        best_start = -1
        best_end = -1
        run_start = -1

        for i, r_cur in enumerate(curr):
            ang = self._beam_angle_deg(i, angle_min, angle_inc)
            if ang < self.sector_min_deg or ang > self.sector_max_deg:
                if run > max_run:
                    max_run = run
                    best_start = run_start
                    best_end = i - 1
                run = 0
                run_start = -1
                continue

            r_prev = prev[i]
            if not self._is_valid_range(r_cur) or not self._is_valid_range(r_prev):
                if run > max_run:
                    max_run = run
                    best_start = run_start
                    best_end = i - 1
                run = 0
                run_start = -1
                continue

            if abs(r_cur - r_prev) >= self.delta_range_m:
                if run == 0:
                    run_start = i
                run += 1
            else:
                if run > max_run:
                    max_run = run
                    best_start = run_start
                    best_end = i - 1
                run = 0
                run_start = -1

        if run > max_run:
            max_run = run
            best_start = run_start
            best_end = len(curr) - 1

        if max_run < self.min_cluster_beams or best_start < 0 or best_end < best_start:
            return False, max_run, None, None

        center_idx = int((best_start + best_end) / 2)
        center_angle_rad = angle_min + center_idx * angle_inc
        center_range_m = float(curr[center_idx])
        return True, max_run, center_angle_rad, center_range_m

    def _find_nearest_target(
        self, curr: List[float], angle_min: float, angle_inc: float
    ) -> Tuple[Optional[float], Optional[float]]:
        best_idx = -1
        best_range = float("inf")
        for i, r_cur in enumerate(curr):
            if not self._is_valid_range(r_cur):
                continue
            ang = self._beam_angle_deg(i, angle_min, angle_inc)
            if ang < self.sector_min_deg or ang > self.sector_max_deg:
                continue
            if r_cur < best_range:
                best_range = r_cur
                best_idx = i

        if best_idx < 0 or not math.isfinite(best_range):
            return None, None
        return angle_min + best_idx * angle_inc, float(best_range)

    def _publish_marker(
        self, scan: LaserScan, detected: bool, angle_rad: Optional[float], range_m: Optional[float]
    ) -> None:
        if not self.publish_marker:
            return

        marker = Marker()
        marker.header = scan.header
        marker.ns = "moving_object"
        marker.id = 1

        if not detected or angle_rad is None or range_m is None:
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
            return

        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.pose.orientation.w = 1.0
        marker.pose.position = Point(
            x=range_m * math.cos(angle_rad),
            y=range_m * math.sin(angle_rad),
            z=0.0,
        )
        marker.scale.x = self.marker_size_m
        marker.scale.y = self.marker_size_m
        marker.scale.z = self.marker_size_m
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 0.95
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(250e6)
        self.marker_pub.publish(marker)

    # -------------------------------------------------------------------------
    # Scan sequence timer
    # SOURCE: scan_sequence_node.cpp style behavior
    # -------------------------------------------------------------------------
    def sequence_timer_cb(self) -> None:
        if self.sequence_dwell > 0:
            self.sequence_dwell -= 1
            return
        self.sequence_dwell = self.sequence_dwell_ticks

        self.sequence_pan_deg += self.sequence_pan_increment_deg * float(self.sequence_pan_dir)
        if self.sequence_pan_deg >= self.sequence_pan_max_deg or self.sequence_pan_deg <= self.sequence_pan_min_deg:
            self.sequence_pan_deg = self._clamp(
                self.sequence_pan_deg, self.sequence_pan_min_deg, self.sequence_pan_max_deg
            )
            self.sequence_pan_dir *= -1

    # -------------------------------------------------------------------------
    # Scan callback
    # SOURCE: object_motion_detector_2d.py
    # -------------------------------------------------------------------------
    def scan_cb(self, msg: LaserScan) -> None:
        self.scan_count += 1
        now = self.get_clock().now()
        curr = list(msg.ranges)

        detected_now = False
        max_cluster = 0
        motion_angle_rad = None
        motion_range_m = None
        track_angle_rad = None
        track_range_m = None
        source = "none"
        pan_motion_block = False

        if self.prev_ranges is not None and len(self.prev_ranges) == len(curr):
            if (
                self.enable_pan_tracking
                and self.prev_scan_pan_deg is not None
                and abs(self.pan_cmd_deg - self.prev_scan_pan_deg) > self.max_pan_change_for_motion_deg
            ):
                pan_motion_block = True
            else:
                detected_now, max_cluster, motion_angle_rad, motion_range_m = self._find_moving_cluster(
                    curr, self.prev_ranges, msg.angle_min, msg.angle_increment
                )
            if motion_angle_rad is not None and motion_range_m is not None:
                track_angle_rad = motion_angle_rad
                track_range_m = motion_range_m
                source = "motion"

        if self.track_with_nearest_fallback and track_angle_rad is None:
            n_ang, n_rng = self._find_nearest_target(curr, msg.angle_min, msg.angle_increment)
            if n_ang is not None and n_rng is not None:
                track_angle_rad = n_ang
                track_range_m = n_rng
                source = "nearest"

        if detected_now:
            self.motion_streak += 1
        else:
            self.motion_streak = max(0, self.motion_streak - 1)

        detected = self.motion_streak >= self.min_motion_frames
        self.detect_pub.publish(Bool(data=detected))

        if detected and track_angle_rad is not None and track_range_m is not None:
            target_angle_deg = track_angle_rad * 180.0 / math.pi
            self.last_target_angle_deg = target_angle_deg
            self.last_target_time = now
            self.target_source = source
            self.angle_pub.publish(Float32(data=target_angle_deg))
            self.range_pub.publish(Float32(data=track_range_m))

        self._publish_marker(
            msg,
            detected and track_angle_rad is not None and track_range_m is not None,
            track_angle_rad,
            track_range_m,
        )

        if detected and not self.last_detect_state:
            dt = (now - self.last_trigger_time).nanoseconds / 1e9
            if dt >= self.trigger_cooldown_sec:
                self.actuate_pub.publish(Bool(data=True))
                self.last_trigger_time = now

        self.last_detect_state = detected
        self.prev_ranges = curr
        self.prev_scan_pan_deg = self.pan_cmd_deg

        if self.debug_enabled and (self.scan_count % self.debug_every_n == 0):
            self.get_logger().info(
                f"MOT scan#{self.scan_count} detected={'yes' if detected else 'no'} "
                f"streak={self.motion_streak} cluster={max_cluster} source={source} "
                f"pan_cmd={self.pan_cmd_deg:.2f} blocked={'yes' if pan_motion_block else 'no'}"
            )

    # -------------------------------------------------------------------------
    # Tracker timer
    # SOURCE: object_pan_tracker_2d.py + object_motion_detector_2d.py
    # -------------------------------------------------------------------------
    def tracker_timer_cb(self) -> None:
        now = self.get_clock().now()
        age_sec = (now - self.last_target_time).nanoseconds / 1e9

        # Baseline desired is hold current command.
        desired_pan = self.pan_cmd_deg

        if (
            self.enable_pan_tracking
            and self.last_target_angle_deg is not None
            and age_sec <= self.target_timeout_sec
            and (not self.track_require_detected or self.last_detect_state)
        ):
            signed_target = -self.last_target_angle_deg if self.invert_angle else self.last_target_angle_deg
            desired_pan = self.pan_center_deg + signed_target
            desired_pan = self._clamp(desired_pan, self.pan_min_deg, self.pan_max_deg)
        elif self.enable_scan_sequence:
            # Optional fallback: sweep when no fresh target.
            desired_pan = self._clamp(self.sequence_pan_deg, self.pan_min_deg, self.pan_max_deg)

        self.pan_desired_deg = desired_pan

        error_deg = desired_pan - self.pan_cmd_deg
        if abs(error_deg) > self.deadband_deg:
            delta = self.k_p * error_deg
            delta = self._clamp(delta, -self.max_step_deg, self.max_step_deg)
            self.pan_cmd_deg = self._clamp(
                self.pan_cmd_deg + delta, self.pan_min_deg, self.pan_max_deg)
        else:
            self.pan_cmd_deg = desired_pan

        # Publish state telemetry.
        self.pan_cmd_pub.publish(Float32(data=float(self.pan_cmd_deg)))
        self.pan_desired_pub.publish(Float32(data=float(self.pan_desired_deg)))

        # Publish servo command (for compatibility with stack).
        servo_msg = Float32MultiArray()
        servo_msg.data = [float(self.pan_cmd_deg), float(self.tilt_deg)]
        self.servo_pub.publish(servo_msg)

        # Send hardware command directly from same file.
        self._send_serial(self.pan_cmd_deg, self.tilt_deg)

        if self.debug_enabled and (self.scan_count % self.debug_every_n == 0):
            self.get_logger().info(
                f"TRK pan_cmd={self.pan_cmd_deg:.2f} pan_des={self.pan_desired_deg:.2f} "
                f"target={self.last_target_angle_deg if self.last_target_angle_deg is not None else 0.0:.2f} "
                f"det={'yes' if self.last_detect_state else 'no'} src={self.target_source} "
                f"invert={'on' if self.invert_angle else 'off'} age={age_sec:.2f}s"
            )

    def destroy_node(self) -> None:
        try:
            if self.serial_conn is not None:
                self.serial_conn.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PanTracker2DSingleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

