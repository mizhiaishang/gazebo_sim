#!/usr/bin/env python3
import csv
import json
import os
import struct
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage


StampedMsg = Tuple[int, object]


@dataclass
class ImuSample:
    index: int
    stamp_ns: int
    ang_vel: Tuple[float, float, float]
    lin_acc: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


@dataclass
class PoseSample:
    stamp_ns: int
    frame_id: str
    child_frame_id: str
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


class DataRecorder(Node):
    def __init__(self) -> None:
        super().__init__("data_recorder")
        self.bridge = CvBridge()

        self.output_root_dir = os.path.expanduser(
            self.declare_parameter("output_dir", "/home/test").get_parameter_value().string_value
        )
        self.run_prefix = self.declare_parameter("run_prefix", "run").get_parameter_value().string_value
        self.save_interval_sec = self.declare_parameter(
            "save_interval_sec", 1.0
        ).get_parameter_value().double_value
        self.save_interval_ns = int(self.save_interval_sec * 1e9)
        self.sync_tolerance_sec = self.declare_parameter(
            "sync_tolerance_sec", 0.6
        ).get_parameter_value().double_value
        self.sync_tolerance_ns = int(self.sync_tolerance_sec * 1e9)
        self.require_lidar = self.declare_parameter(
            "require_lidar", True
        ).get_parameter_value().bool_value

        self.imu_save_hz = float(
            self.declare_parameter("imu_save_hz", 20.0).get_parameter_value().double_value
        )
        if self.imu_save_hz <= 0.0:
            self.imu_save_hz = 20.0
        self.imu_period_ns = max(1, int(1e9 / self.imu_save_hz))
        self.imu_window_sec = float(
            self.declare_parameter("imu_window_sec", 0.1).get_parameter_value().double_value
        )
        if self.imu_window_sec < 0.0:
            self.imu_window_sec = 0.1
        self.imu_align_tolerance_sec = float(
            self.declare_parameter("imu_align_tolerance_sec", 0.01).get_parameter_value().double_value
        )
        self.imu_align_tolerance_ns = int(self.imu_align_tolerance_sec * 1e9)

        self.pose_topic = self.declare_parameter(
            "pose_topic", "/world/small_house/dynamic_pose/info"
        ).get_parameter_value().string_value
        pose_children_raw = self.declare_parameter(
            "pose_child_frames", "smart_agent,smart_agent/base_link,smart_agent::base_link"
        ).get_parameter_value().string_value
        self.pose_child_candidates = tuple(
            s.strip().lstrip("/") for s in pose_children_raw.split(",") if s.strip()
        )
        self.pose_align_tolerance_sec = float(
            self.declare_parameter("pose_align_tolerance_sec", 0.02).get_parameter_value().double_value
        )
        self.pose_align_tolerance_ns = int(self.pose_align_tolerance_sec * 1e9)

        self.buffer_size = int(self.declare_parameter("buffer_size", 1200).get_parameter_value().integer_value)
        if self.buffer_size < 100:
            self.buffer_size = 100
        self.imu_buffer_size = int(
            self.declare_parameter("imu_buffer_size", 4000).get_parameter_value().integer_value
        )
        if self.imu_buffer_size < 400:
            self.imu_buffer_size = 400

        self.buffers: Dict[str, Deque[StampedMsg]] = {
            "front_rgb": deque(maxlen=self.buffer_size),
            "rear_rgb": deque(maxlen=self.buffer_size),
            "depth": deque(maxlen=self.buffer_size),
            "lidar_points": deque(maxlen=self.buffer_size),
        }
        self.imu_stream_buffer: Deque[ImuSample] = deque(maxlen=self.imu_buffer_size)
        self.pose_buffer: Deque[PoseSample] = deque(maxlen=self.imu_buffer_size)

        self.last_trigger_ns: Optional[int] = None
        self.last_saved_target_ns: Optional[int] = None
        self.imu_stream_index = 0
        self.last_imu_saved_ns: Optional[int] = None
        self.imu_source_topics: Dict[str, Tuple[object, object]] = {}
        self.imu_reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.imu_messages_seen = 0
        self._warned_lidar_missing = False
        self._warned_pose_missing = False
        self._warned_imu_missing = False
        self.system_timer_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        os.makedirs(self.output_root_dir, exist_ok=True)
        self.run_name, self.run_dir = self._create_run_dir()
        self.steps_dir = os.path.join(self.run_dir, "steps")
        os.makedirs(self.steps_dir, exist_ok=True)
        self.step_index = 1

        self.imu_stream_path = os.path.join(self.run_dir, "imu_stream.csv")
        self._imu_stream_file = open(self.imu_stream_path, "w", newline="", encoding="utf-8")
        self._imu_writer = csv.writer(self._imu_stream_file)
        self._imu_writer.writerow(
            [
                "index",
                "time_ns",
                "ang_vel_x",
                "ang_vel_y",
                "ang_vel_z",
                "lin_acc_x",
                "lin_acc_y",
                "lin_acc_z",
                "ori_x",
                "ori_y",
                "ori_z",
                "ori_w",
                "pose_x",
                "pose_y",
                "pose_z",
            ]
        )
        self._imu_stream_file.flush()

        self._write_run_meta()
        self._create_subscribers()
        self.create_timer(self.save_interval_sec, self._save_aligned_snapshot, clock=self.system_timer_clock)
        self.create_timer(2.0, self._discover_imu_topics, clock=self.system_timer_clock)

        self.get_logger().info(
            "Data recorder started. "
            f"run_dir={self.run_dir}, save_interval_sec={self.save_interval_sec}, "
            f"sync_tolerance_sec={self.sync_tolerance_sec}, require_lidar={self.require_lidar}, "
            f"imu_save_hz={self.imu_save_hz}, imu_window_sec={self.imu_window_sec}, "
            f"pose_topic={self.pose_topic}"
        )

    def _create_run_dir(self) -> Tuple[str, str]:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"{self.run_prefix}_{ts}"
        run_name = base
        run_dir = os.path.join(self.output_root_dir, run_name)
        idx = 1
        while os.path.exists(run_dir):
            run_name = f"{base}_{idx:02d}"
            run_dir = os.path.join(self.output_root_dir, run_name)
            idx += 1
        os.makedirs(run_dir, exist_ok=True)
        return run_name, run_dir

    def _write_run_meta(self) -> None:
        data = {
            "run_name": self.run_name,
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "output_root_dir": self.output_root_dir,
            "settings": {
                "save_interval_sec": self.save_interval_sec,
                "sync_tolerance_sec": self.sync_tolerance_sec,
                "require_lidar": self.require_lidar,
                "imu_save_hz": self.imu_save_hz,
                "imu_window_sec": self.imu_window_sec,
                "imu_align_tolerance_sec": self.imu_align_tolerance_sec,
                "pose_topic": self.pose_topic,
                "pose_child_candidates": list(self.pose_child_candidates),
                "pose_align_tolerance_sec": self.pose_align_tolerance_sec,
            },
            "files": {
                "imu_stream": "imu_stream.csv",
                "steps_dir": "steps",
            },
        }
        with open(os.path.join(self.run_dir, "run_meta.json"), "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    def _create_subscribers(self) -> None:
        self.create_subscription(Image, "/agent/front_camera/image_raw", self._front_cb, 20)
        self.create_subscription(Image, "/agent/rear_camera/image_raw", self._rear_cb, 20)
        self.create_subscription(Image, "/agent/depth_camera/image_raw", self._depth_cb, 20)
        self.create_subscription(PointCloud2, "/agent/lidar/scan/points", self._lidar_points_cb, 20)
        self._add_imu_subscription("/agent/imu")
        self._add_imu_subscription("/agent/imu_scoped")
        self._add_imu_subscription("/agent/imu_scoped_world")
        self._add_imu_subscription("/agent/imu_scoped_model")
        self._add_imu_subscription("/imu")
        self.create_subscription(TFMessage, self.pose_topic, self._pose_cb, 20)

    def _imu_cb_with_topic(self, topic: str) -> Callable[[Imu], None]:
        def _cb(msg: Imu) -> None:
            self._imu_cb(msg, topic)

        return _cb

    def _add_imu_subscription(self, topic: str) -> None:
        if topic in self.imu_source_topics:
            return
        sub_reliable = self.create_subscription(
            Imu,
            topic,
            self._imu_cb_with_topic(f"{topic} [reliable]"),
            self.imu_reliable_qos,
        )
        sub_best_effort = self.create_subscription(
            Imu,
            topic,
            self._imu_cb_with_topic(f"{topic} [best_effort]"),
            qos_profile_sensor_data,
        )
        self.imu_source_topics[topic] = (sub_reliable, sub_best_effort)
        self.get_logger().info(f"IMU subscriber attached (reliable+best_effort): {topic}")

    def _discover_imu_topics(self) -> None:
        try:
            for topic, types in self.get_topic_names_and_types():
                if "sensor_msgs/msg/Imu" not in types:
                    continue
                self._add_imu_subscription(topic)
        except Exception as exc:
            self.get_logger().warn(f"IMU topic discovery failed once: {exc}")

    def _stamp_to_ns(self, sec: int, nanosec: int) -> int:
        return int(sec) * 1_000_000_000 + int(nanosec)

    def _msg_stamp_ns(self, msg: object) -> int:
        header = getattr(msg, "header", None)
        if header is not None:
            stamp = getattr(header, "stamp", None)
            if stamp is not None:
                sec = int(getattr(stamp, "sec", 0))
                nanosec = int(getattr(stamp, "nanosec", 0))
                if sec != 0 or nanosec != 0:
                    return self._stamp_to_ns(sec, nanosec)
        ros_now_ns = self.get_clock().now().nanoseconds
        if ros_now_ns > 0:
            return ros_now_ns
        return self.system_timer_clock.now().nanoseconds

    def _normalize_frame(self, frame_id: str) -> str:
        return frame_id.strip().lstrip("/")

    def _push(self, key: str, msg: object) -> None:
        self.buffers[key].append((self._msg_stamp_ns(msg), msg))

    def _front_cb(self, msg: Image) -> None:
        self._push("front_rgb", msg)

    def _rear_cb(self, msg: Image) -> None:
        self._push("rear_rgb", msg)

    def _depth_cb(self, msg: Image) -> None:
        self._push("depth", msg)

    def _lidar_points_cb(self, msg: PointCloud2) -> None:
        self._push("lidar_points", msg)

    def _imu_cb(self, msg: Imu, source_topic: str = "unknown") -> None:
        stamp_ns = self._msg_stamp_ns(msg)
        if self.last_imu_saved_ns is not None and stamp_ns >= self.last_imu_saved_ns:
            if stamp_ns - self.last_imu_saved_ns < self.imu_period_ns:
                return

        self.last_imu_saved_ns = stamp_ns
        self.imu_stream_index += 1
        self.imu_messages_seen += 1
        if self.imu_messages_seen == 1:
            self.get_logger().info(f"First IMU message received from topic: {source_topic}")
        sample = ImuSample(
            index=self.imu_stream_index,
            stamp_ns=stamp_ns,
            ang_vel=(
                float(msg.angular_velocity.x),
                float(msg.angular_velocity.y),
                float(msg.angular_velocity.z),
            ),
            lin_acc=(
                float(msg.linear_acceleration.x),
                float(msg.linear_acceleration.y),
                float(msg.linear_acceleration.z),
            ),
            orientation=(
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            ),
        )
        self.imu_stream_buffer.append(sample)
        pose_sample, _ = self._nearest_pose_sample(stamp_ns)
        if pose_sample is None:
            pose_xyz = (float("nan"), float("nan"), float("nan"))
        else:
            pose_xyz = (
                pose_sample.position[0],
                pose_sample.position[1],
                pose_sample.position[2],
            )
        self._imu_writer.writerow(
            [
                sample.index,
                sample.stamp_ns,
                sample.ang_vel[0],
                sample.ang_vel[1],
                sample.ang_vel[2],
                sample.lin_acc[0],
                sample.lin_acc[1],
                sample.lin_acc[2],
                sample.orientation[0],
                sample.orientation[1],
                sample.orientation[2],
                sample.orientation[3],
                pose_xyz[0],
                pose_xyz[1],
                pose_xyz[2],
            ]
        )
        self._imu_stream_file.flush()

    def _pose_cb(self, msg: TFMessage) -> None:
        transform = self._select_pose_transform(msg.transforms)
        if transform is None:
            return
        stamp = transform.header.stamp
        stamp_ns = self._stamp_to_ns(stamp.sec, stamp.nanosec)
        if stamp_ns == 0:
            stamp_ns = self.get_clock().now().nanoseconds
        pose = PoseSample(
            stamp_ns=stamp_ns,
            frame_id=self._normalize_frame(transform.header.frame_id),
            child_frame_id=self._normalize_frame(transform.child_frame_id),
            position=(
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                float(transform.transform.translation.z),
            ),
            orientation=(
                float(transform.transform.rotation.x),
                float(transform.transform.rotation.y),
                float(transform.transform.rotation.z),
                float(transform.transform.rotation.w),
            ),
        )
        self.pose_buffer.append(pose)

    def _select_pose_transform(self, transforms: Sequence[object]) -> Optional[object]:
        if not transforms:
            return None
        for candidate in self.pose_child_candidates:
            for transform in transforms:
                child = self._normalize_frame(getattr(transform, "child_frame_id", ""))
                if child == candidate:
                    return transform
        for transform in transforms:
            child = self._normalize_frame(getattr(transform, "child_frame_id", ""))
            if "smart_agent" in child:
                return transform
        return None

    def _nearest(self, key: str, target_ns: int) -> Tuple[Optional[int], Optional[object], Optional[int]]:
        items = self.buffers[key]
        if not items:
            return None, None, None

        best_stamp = None
        best_msg = None
        best_abs_dt = None
        for stamp_ns, msg in items:
            dt = abs(stamp_ns - target_ns)
            if best_abs_dt is None or dt < best_abs_dt:
                best_stamp = stamp_ns
                best_msg = msg
                best_abs_dt = dt
        return best_stamp, best_msg, best_abs_dt

    def _nearest_imu_sample(self, target_ns: int) -> Tuple[Optional[ImuSample], Optional[int]]:
        if not self.imu_stream_buffer:
            return None, None
        best_sample = None
        best_abs_dt = None
        for sample in self.imu_stream_buffer:
            dt = abs(sample.stamp_ns - target_ns)
            if best_abs_dt is None or dt < best_abs_dt:
                best_sample = sample
                best_abs_dt = dt
        return best_sample, best_abs_dt

    def _nearest_pose_sample(self, target_ns: int) -> Tuple[Optional[PoseSample], Optional[int]]:
        if not self.pose_buffer:
            return None, None
        best_sample = None
        best_abs_dt = None
        for sample in self.pose_buffer:
            dt = abs(sample.stamp_ns - target_ns)
            if best_abs_dt is None or dt < best_abs_dt:
                best_sample = sample
                best_abs_dt = dt
        return best_sample, best_abs_dt

    def _build_imu_window(self, rgb_time_ns: int, frame_dir: str) -> Dict[str, object]:
        imu_dir = os.path.join(frame_dir, "imu")
        os.makedirs(imu_dir, exist_ok=True)
        window_path = os.path.join(imu_dir, "window.npy")
        window_rel_path = "imu/window.npy"

        if not self.imu_stream_buffer:
            arr = np.empty((0, 13), dtype=np.float64)
            np.save(window_path, arr)
            return {
                "path": window_rel_path,
                "sample_hz": self.imu_save_hz,
                "count": 0,
                "valid_count": 0,
                "start_ns": rgb_time_ns,
                "end_ns": rgb_time_ns,
            }

        half_count = int(round(self.imu_window_sec * self.imu_save_hz))
        if half_count < 0:
            half_count = 0
        sample_count = 2 * half_count + 1
        rows: List[List[float]] = []
        valid_count = 0
        start_ns = rgb_time_ns - half_count * self.imu_period_ns
        end_ns = rgb_time_ns + half_count * self.imu_period_ns

        for i in range(-half_count, half_count + 1):
            target_ns = rgb_time_ns + i * self.imu_period_ns
            sample, abs_dt = self._nearest_imu_sample(target_ns)
            rel_ms = i * (1000.0 / self.imu_save_hz)
            if sample is None or abs_dt is None:
                rows.append(
                    [
                        rel_ms,
                        np.nan,
                        np.nan,
                        np.nan,
                        np.nan,
                        np.nan,
                        np.nan,
                        np.nan,
                        np.nan,
                        np.nan,
                        np.nan,
                        -1.0,
                        np.nan,
                    ]
                )
                continue
            valid_count += 1
            rows.append(
                [
                    rel_ms,
                    sample.ang_vel[0],
                    sample.ang_vel[1],
                    sample.ang_vel[2],
                    sample.lin_acc[0],
                    sample.lin_acc[1],
                    sample.lin_acc[2],
                    sample.orientation[0],
                    sample.orientation[1],
                    sample.orientation[2],
                    sample.orientation[3],
                    float(sample.index),
                    float(sample.stamp_ns - rgb_time_ns) / 1e6,
                ]
            )

        arr = np.array(rows, dtype=np.float64).reshape(sample_count, 13)
        np.save(window_path, arr)
        return {
            "path": window_rel_path,
            "sample_hz": self.imu_save_hz,
            "count": sample_count,
            "valid_count": valid_count,
            "start_ns": start_ns,
            "end_ns": end_ns,
        }

    def _save_aligned_snapshot(self) -> None:
        target_ns = self.get_clock().now().nanoseconds
        if self.last_trigger_ns is None:
            self.last_trigger_ns = target_ns
            return
        if target_ns < self.last_trigger_ns:
            self.last_trigger_ns = target_ns
            return
        if target_ns - self.last_trigger_ns < self.save_interval_ns:
            return
        self.last_trigger_ns = target_ns
        if self.last_saved_target_ns is not None and target_ns == self.last_saved_target_ns:
            return

        selected: Dict[str, Tuple[int, object, int]] = {}
        for key in ("front_rgb", "rear_rgb", "depth"):
            stamp_ns, msg, abs_dt = self._nearest(key, target_ns)
            if stamp_ns is None or msg is None or abs_dt is None:
                self.get_logger().warn(f"Skip snapshot: {key} buffer is empty")
                return
            if abs_dt > self.sync_tolerance_ns:
                self.get_logger().warn(
                    f"Skip snapshot: {key} nearest frame too old/new "
                    f"(|dt|={abs_dt / 1e6:.1f}ms, tol={self.sync_tolerance_ns / 1e6:.1f}ms)"
                )
                return
            selected[key] = (stamp_ns, msg, abs_dt)

        lidar_points = self._nearest("lidar_points", target_ns)
        chosen_lidar = None

        # Fixed format: only accept PointCloud2 and store float32 Nx4 [x, y, z, intensity].
        p_stamp_ns, p_msg, p_abs_dt = lidar_points
        if (
            p_stamp_ns is not None
            and p_msg is not None
            and p_abs_dt is not None
            and p_abs_dt <= self.sync_tolerance_ns
        ):
            chosen_lidar = (p_stamp_ns, p_msg, p_abs_dt)

        if chosen_lidar is None and self.require_lidar:
            self.get_logger().warn("Skip snapshot: lidar points unavailable in tolerance window")
            return

        lidar_available = chosen_lidar is not None
        if lidar_available:
            selected["lidar"] = chosen_lidar  # type: ignore[assignment]
            self._warned_lidar_missing = False
        elif not self._warned_lidar_missing:
            self.get_logger().warn("Lidar missing for this step; rgb/depth will still be written.")
            self._warned_lidar_missing = True

        rgb_time_ns = selected["front_rgb"][0]
        imu_anchor, imu_anchor_dt = self._nearest_imu_sample(rgb_time_ns)
        pose_sample, pose_dt = self._nearest_pose_sample(rgb_time_ns)

        frame_name = f"step{self.step_index:06d}"
        frame_dir = os.path.join(self.steps_dir, frame_name)
        while os.path.exists(frame_dir):
            self.step_index += 1
            frame_name = f"step{self.step_index:06d}"
            frame_dir = os.path.join(self.steps_dir, frame_name)
        os.makedirs(frame_dir, exist_ok=True)

        front_dir = os.path.join(frame_dir, "front_rgb")
        rear_dir = os.path.join(frame_dir, "rear_rgb")
        depth_dir = os.path.join(frame_dir, "depth")
        os.makedirs(front_dir, exist_ok=True)
        os.makedirs(rear_dir, exist_ok=True)
        os.makedirs(depth_dir, exist_ok=True)

        self._save_rgb(selected["front_rgb"][1], os.path.join(front_dir, "image.png"))
        self._save_rgb(selected["rear_rgb"][1], os.path.join(rear_dir, "image.png"))
        self._save_depth(selected["depth"][1], os.path.join(depth_dir, "image.png"))

        lidar_meta: Dict[str, object]
        if lidar_available:
            lidar_dir = os.path.join(frame_dir, "lidar")
            os.makedirs(lidar_dir, exist_ok=True)
            lidar_path = os.path.join(lidar_dir, "scan.bin")
            lidar_format = self._save_lidar(selected["lidar"][1], lidar_path)
            lidar_meta = {
                "available": True,
                "msg_time_ns": selected["lidar"][0],
                "delta_ms": round(selected["lidar"][2] / 1e6, 3),
                "path": "lidar/scan.bin",
                "type": "points",
                "format": lidar_format,
            }
        else:
            lidar_meta = {
                "available": False,
                "reason": "points unavailable in tolerance window",
            }

        imu_window_meta = self._build_imu_window(rgb_time_ns, frame_dir)

        if imu_anchor is None or imu_anchor_dt is None:
            imu_anchor_meta = {
                "available": False,
                "reason": "imu stream empty",
            }
            imu_valid = False
            if not self._warned_imu_missing:
                self.get_logger().warn("IMU stream empty; saved RGB without aligned IMU anchor.")
                self._warned_imu_missing = True
        else:
            imu_anchor_meta = {
                "available": True,
                "stream_index": imu_anchor.index,
                "time_ns": imu_anchor.stamp_ns,
                "delta_ms": round(imu_anchor_dt / 1e6, 3),
            }
            imu_valid = imu_anchor_dt <= self.imu_align_tolerance_ns
            self._warned_imu_missing = False

        if pose_sample is None or pose_dt is None:
            pose_gt_meta = {
                "available": False,
                "reason": f"no pose message on topic {self.pose_topic}",
            }
            pose_valid = False
            if not self._warned_pose_missing:
                self.get_logger().warn(
                    f"Ground-truth pose unavailable on {self.pose_topic}; saving RGB without pose."
                )
                self._warned_pose_missing = True
        else:
            pose_gt_meta = {
                "available": True,
                "time_ns": pose_sample.stamp_ns,
                "delta_ms": round(pose_dt / 1e6, 3),
                "frame_id": pose_sample.frame_id,
                "child_frame_id": pose_sample.child_frame_id,
                "position": {
                    "x": pose_sample.position[0],
                    "y": pose_sample.position[1],
                    "z": pose_sample.position[2],
                },
                "orientation": {
                    "x": pose_sample.orientation[0],
                    "y": pose_sample.orientation[1],
                    "z": pose_sample.orientation[2],
                    "w": pose_sample.orientation[3],
                },
            }
            pose_valid = pose_dt <= self.pose_align_tolerance_ns
            self._warned_pose_missing = False

        metadata = {
            "step": frame_name,
            "rgb_time_ns": rgb_time_ns,
            "trigger_time_ns": target_ns,
            "alignment_valid": {
                "imu": imu_valid,
                "pose_gt": pose_valid,
                "lidar": bool(lidar_meta.get("available", False)),
            },
            "sensors": {
                "front_rgb": {
                    "msg_time_ns": selected["front_rgb"][0],
                    "delta_ms": round(selected["front_rgb"][2] / 1e6, 3),
                    "path": "front_rgb/image.png",
                },
                "rear_rgb": {
                    "msg_time_ns": selected["rear_rgb"][0],
                    "delta_ms": round(selected["rear_rgb"][2] / 1e6, 3),
                    "path": "rear_rgb/image.png",
                },
                "depth": {
                    "msg_time_ns": selected["depth"][0],
                    "delta_ms": round(selected["depth"][2] / 1e6, 3),
                    "path": "depth/image.png",
                },
                "lidar": lidar_meta,
            },
            "imu_anchor": imu_anchor_meta,
            "imu_window": imu_window_meta,
            "pose_gt": pose_gt_meta,
        }
        with open(os.path.join(frame_dir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2)

        self.last_saved_target_ns = target_ns
        self.step_index += 1

        lidar_delta = metadata["sensors"]["lidar"].get("delta_ms", "NA")
        imu_delta = metadata["imu_anchor"].get("delta_ms", "NA")
        pose_delta = metadata["pose_gt"].get("delta_ms", "NA")
        self.get_logger().info(
            f"Saved frame {frame_name} "
            f"(rgb={rgb_time_ns}, lidar={lidar_delta}ms, imu={imu_delta}ms, pose_gt={pose_delta}ms)"
        )

    def _save_rgb(self, msg: Image, path: str) -> None:
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(path, image)

    def _save_depth(self, msg: Image, path: str) -> None:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth.dtype == np.float32:
            depth_mm = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
            depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)
        elif depth.dtype == np.uint16:
            depth_mm = depth
        else:
            depth_mm = depth.astype(np.uint16)
        cv2.imwrite(path, depth_mm)

    def _save_lidar(self, msg: object, path: str) -> str:
        if not isinstance(msg, PointCloud2):
            np.array([], dtype=np.float32).tofile(path)
            return "empty"

        field_map = {f.name: f for f in msg.fields}
        x_field = field_map.get("x")
        y_field = field_map.get("y")
        z_field = field_map.get("z")
        intensity_field = field_map.get("intensity") or field_map.get("i")
        if x_field is None or y_field is None or z_field is None:
            np.array([], dtype=np.float32).tofile(path)
            return "empty (missing x/y/z fields in PointCloud2)"

        endian = ">" if msg.is_bigendian else "<"

        def _read_field(point_base: int, field: PointField) -> float:
            off = point_base + int(field.offset)
            dt = int(field.datatype)
            if dt == PointField.FLOAT32:
                return float(struct.unpack_from(endian + "f", msg.data, off)[0])
            if dt == PointField.FLOAT64:
                return float(struct.unpack_from(endian + "d", msg.data, off)[0])
            if dt == PointField.INT8:
                return float(struct.unpack_from(endian + "b", msg.data, off)[0])
            if dt == PointField.UINT8:
                return float(struct.unpack_from(endian + "B", msg.data, off)[0])
            if dt == PointField.INT16:
                return float(struct.unpack_from(endian + "h", msg.data, off)[0])
            if dt == PointField.UINT16:
                return float(struct.unpack_from(endian + "H", msg.data, off)[0])
            if dt == PointField.INT32:
                return float(struct.unpack_from(endian + "i", msg.data, off)[0])
            if dt == PointField.UINT32:
                return float(struct.unpack_from(endian + "I", msg.data, off)[0])
            return float("nan")

        rows: List[Tuple[float, float, float, float]] = []
        width = int(msg.width)
        height = int(msg.height)
        point_step = int(msg.point_step)
        row_step = int(msg.row_step)
        for r in range(height):
            row_base = r * row_step
            for c in range(width):
                base = row_base + c * point_step
                x = _read_field(base, x_field)
                y = _read_field(base, y_field)
                z = _read_field(base, z_field)
                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                    continue
                intensity = 0.0
                if intensity_field is not None:
                    val = _read_field(base, intensity_field)
                    intensity = float(val) if np.isfinite(val) else 0.0
                rows.append((x, y, z, intensity))

        arr = np.asarray(rows, dtype=np.float32)
        arr.tofile(path)
        return (
            "float32 Nx4 [x, y, z, intensity]; "
            f"count={arr.shape[0]}; source_point_step={msg.point_step}, source_row_step={msg.row_step}"
        )

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "_imu_stream_file") and self._imu_stream_file:
                self._imu_stream_file.flush()
                self._imu_stream_file.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
