#!/usr/bin/env python3
import csv
import os
import struct
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from tf2_msgs.msg import TFMessage


StampedMsg = Tuple[int, object]


@dataclass
class PoseSample:
    stamp_ns: int
    frame_id: str
    child_frame_id: str
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


class DatasetRecorder(Node):
    def __init__(self) -> None:
        super().__init__("dataset_recorder")
        self.bridge = CvBridge()

        self.output_root_dir = os.path.expanduser(
            self.declare_parameter(
                "output_dir", "/home/test/dataset"
            ).get_parameter_value().string_value
        )
        self.save_interval_sec = float(
            self.declare_parameter("save_interval_sec", 1.0).get_parameter_value().double_value
        )
        self.save_interval_ns = int(self.save_interval_sec * 1e9)
        self.sync_tolerance_sec = float(
            self.declare_parameter("sync_tolerance_sec", 0.6).get_parameter_value().double_value
        )
        self.sync_tolerance_ns = int(self.sync_tolerance_sec * 1e9)
        self.require_lidar = bool(
            self.declare_parameter("require_lidar", True).get_parameter_value().bool_value
        )

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

        self.buffers: Dict[str, Deque[StampedMsg]] = {
            "front_rgb": deque(maxlen=self.buffer_size),
            "rear_rgb": deque(maxlen=self.buffer_size),
            "lidar_points": deque(maxlen=self.buffer_size),
        }
        self.pose_buffer: Deque[PoseSample] = deque(maxlen=self.buffer_size)

        self.last_trigger_ns: Optional[int] = None
        self.last_saved_front_ns: Optional[int] = None
        self.system_timer_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        self._warned_lidar_missing = False
        self._warned_pose_missing = False

        self.front_cam_dir = os.path.join(self.output_root_dir, "front_cam")
        self.back_cam_dir = os.path.join(self.output_root_dir, "back_cam")
        self.lidar_dir = os.path.join(self.output_root_dir, "lidar")
        os.makedirs(self.front_cam_dir, exist_ok=True)
        os.makedirs(self.back_cam_dir, exist_ok=True)
        os.makedirs(self.lidar_dir, exist_ok=True)

        self.track_csv_path = os.path.join(self.output_root_dir, "track.csv")
        self._open_track_csv()

        self._create_subscribers()
        self.create_timer(self.save_interval_sec, self._save_dataset_snapshot, clock=self.system_timer_clock)

        self.get_logger().info(
            "Dataset recorder started. "
            f"output_dir={self.output_root_dir}, save_interval_sec={self.save_interval_sec}, "
            f"sync_tolerance_sec={self.sync_tolerance_sec}, require_lidar={self.require_lidar}, "
            f"pose_topic={self.pose_topic}"
        )

    def _open_track_csv(self) -> None:
        file_exists = os.path.exists(self.track_csv_path)
        self._track_file = open(self.track_csv_path, "a", newline="", encoding="utf-8")
        self._track_writer = csv.writer(self._track_file)
        if (not file_exists) or os.path.getsize(self.track_csv_path) == 0:
            self._track_writer.writerow(
                [
                    "timestamp",
                    "front_cam_ts",
                    "back_cam_ts",
                    "lidar_ts",
                    "tx",
                    "ty",
                    "tz",
                    "qx",
                    "qy",
                    "qz",
                    "qw",
                ]
            )
            self._track_file.flush()

    def _create_subscribers(self) -> None:
        self.create_subscription(Image, "/agent/front_camera/image_raw", self._front_cb, 20)
        self.create_subscription(Image, "/agent/rear_camera/image_raw", self._rear_cb, 20)
        self.create_subscription(PointCloud2, "/agent/lidar/scan/points", self._lidar_points_cb, 20)
        self.create_subscription(TFMessage, self.pose_topic, self._pose_cb, 20)

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

    def _lidar_points_cb(self, msg: PointCloud2) -> None:
        self._push("lidar_points", msg)

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

    def _save_dataset_snapshot(self) -> None:
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

        selected: Dict[str, Tuple[int, object, int]] = {}
        for key in ("front_rgb", "rear_rgb"):
            stamp_ns, msg, abs_dt = self._nearest(key, target_ns)
            if stamp_ns is None or msg is None or abs_dt is None:
                self.get_logger().warn(f"Skip dataset snapshot: {key} buffer is empty")
                return
            if abs_dt > self.sync_tolerance_ns:
                self.get_logger().warn(
                    f"Skip dataset snapshot: {key} outside tolerance (|dt|={abs_dt / 1e6:.1f}ms)"
                )
                return
            selected[key] = (stamp_ns, msg, abs_dt)

        l_stamp_ns, l_msg, l_abs_dt = self._nearest("lidar_points", target_ns)
        if (
            l_stamp_ns is None
            or l_msg is None
            or l_abs_dt is None
            or l_abs_dt > self.sync_tolerance_ns
        ):
            if self.require_lidar:
                self.get_logger().warn("Skip dataset snapshot: lidar points unavailable in tolerance window")
                return
            if not self._warned_lidar_missing:
                self.get_logger().warn("Dataset snapshot without lidar is disabled by format; skipping.")
                self._warned_lidar_missing = True
            return

        selected["lidar"] = (l_stamp_ns, l_msg, l_abs_dt)
        self._warned_lidar_missing = False

        front_ts = selected["front_rgb"][0]
        rear_ts = selected["rear_rgb"][0]
        lidar_ts = selected["lidar"][0]
        if self.last_saved_front_ns is not None and front_ts == self.last_saved_front_ns:
            return

        pose_sample, pose_dt = self._nearest_pose_sample(front_ts)
        if pose_sample is None or pose_dt is None:
            if not self._warned_pose_missing:
                self.get_logger().warn(
                    f"Skip dataset snapshot: no pose on {self.pose_topic} for front_ts={front_ts}"
                )
                self._warned_pose_missing = True
            return
        if pose_dt > self.pose_align_tolerance_ns:
            self.get_logger().warn(
                f"Skip dataset snapshot: pose misaligned (|dt|={pose_dt / 1e6:.1f}ms, tol={self.pose_align_tolerance_ns / 1e6:.1f}ms)"
            )
            return
        self._warned_pose_missing = False

        front_path = os.path.join(self.front_cam_dir, f"{front_ts}.png")
        rear_path = os.path.join(self.back_cam_dir, f"{rear_ts}.png")
        lidar_path = os.path.join(self.lidar_dir, f"{lidar_ts}.bin")

        self._save_rgb(selected["front_rgb"][1], front_path)
        self._save_rgb(selected["rear_rgb"][1], rear_path)
        self._save_lidar(selected["lidar"][1], lidar_path)

        self._track_writer.writerow(
            [
                int(front_ts),
                int(front_ts),
                int(rear_ts),
                int(lidar_ts),
                pose_sample.position[0],
                pose_sample.position[1],
                pose_sample.position[2],
                pose_sample.orientation[0],
                pose_sample.orientation[1],
                pose_sample.orientation[2],
                pose_sample.orientation[3],
            ]
        )
        self._track_file.flush()
        self.last_saved_front_ns = front_ts

        self.get_logger().info(
            "Saved dataset sample "
            f"(ts={front_ts}, rear_dt={selected['rear_rgb'][2] / 1e6:.1f}ms, "
            f"lidar_dt={selected['lidar'][2] / 1e6:.1f}ms, pose_dt={pose_dt / 1e6:.1f}ms)"
        )

    def _save_rgb(self, msg: Image, path: str) -> None:
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(path, image)

    def _save_lidar(self, msg: object, path: str) -> None:
        if not isinstance(msg, PointCloud2):
            np.array([], dtype=np.float32).tofile(path)
            return

        field_map = {f.name: f for f in msg.fields}
        x_field = field_map.get("x")
        y_field = field_map.get("y")
        z_field = field_map.get("z")
        intensity_field = field_map.get("intensity") or field_map.get("i")
        if x_field is None or y_field is None or z_field is None:
            np.array([], dtype=np.float32).tofile(path)
            return

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

        rows = []
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

        np.asarray(rows, dtype=np.float32).tofile(path)

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "_track_file") and self._track_file:
                self._track_file.flush()
                self._track_file.close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
