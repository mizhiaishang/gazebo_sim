#!/usr/bin/env python3
import json
import os
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2


StampedMsg = Tuple[int, object]


class DataRecorder(Node):
    def __init__(self) -> None:
        super().__init__('data_recorder')
        self.bridge = CvBridge()

        self.output_dir = os.path.expanduser(
            self.declare_parameter('output_dir', '~/agent_records').get_parameter_value().string_value
        )
        self.save_interval_sec = self.declare_parameter(
            'save_interval_sec', 2.0
        ).get_parameter_value().double_value
        self.save_interval_ns = int(self.save_interval_sec * 1e9)
        self.sync_tolerance_sec = self.declare_parameter(
            'sync_tolerance_sec', 0.6
        ).get_parameter_value().double_value
        self.allow_missing_lidar = bool(
            self.declare_parameter('allow_missing_lidar', False).get_parameter_value().bool_value
        )
        self.buffer_size = int(
            self.declare_parameter('buffer_size', 300).get_parameter_value().integer_value
        )

        self.buffers: Dict[str, Deque[StampedMsg]] = {
            'front_rgb': deque(maxlen=self.buffer_size),
            'rear_rgb': deque(maxlen=self.buffer_size),
            'depth': deque(maxlen=self.buffer_size),
            'lidar_scan': deque(maxlen=self.buffer_size),
            'lidar_points': deque(maxlen=self.buffer_size),
        }
        self.last_trigger_ns: Optional[int] = None
        self.last_saved_target_ns: Optional[int] = None

        os.makedirs(self.output_dir, exist_ok=True)
        self.step_index = self._get_next_step_index()
        self._create_subscribers()
        self.system_timer_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.create_timer(self.save_interval_sec, self._save_aligned_snapshot, clock=self.system_timer_clock)

        self.get_logger().info(
            'Data recorder started. '
            f'output_dir={self.output_dir}, save_interval_sec={self.save_interval_sec}, '
            f'sync_tolerance_sec={self.sync_tolerance_sec}, allow_missing_lidar={self.allow_missing_lidar}, '
            f'next_step=step{self.step_index}'
        )

    def _get_next_step_index(self) -> int:
        max_idx = 0
        try:
            for name in os.listdir(self.output_dir):
                path = os.path.join(self.output_dir, name)
                if not os.path.isdir(path):
                    continue
                if not name.startswith('step'):
                    continue
                suffix = name[4:]
                if suffix.isdigit():
                    max_idx = max(max_idx, int(suffix))
        except OSError:
            pass
        return max_idx + 1

    def _create_subscribers(self) -> None:
        self.create_subscription(Image, '/agent/front_camera/image_raw', self._front_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/agent/rear_camera/image_raw', self._rear_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/agent/depth_camera/image_raw', self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/agent/lidar/scan', self._lidar_scan_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/agent/lidar/scan/points', self._lidar_points_cb, qos_profile_sensor_data)

    def _msg_stamp_ns(self, msg: object) -> int:
        header = getattr(msg, 'header', None)
        if header is not None:
            sec = int(getattr(header.stamp, 'sec', 0))
            nanosec = int(getattr(header.stamp, 'nanosec', 0))
            if sec != 0 or nanosec != 0:
                return sec * 1_000_000_000 + nanosec
        return self.get_clock().now().nanoseconds

    def _push(self, key: str, msg: object) -> None:
        self.buffers[key].append((self._msg_stamp_ns(msg), msg))

    def _front_cb(self, msg: Image) -> None:
        self._push('front_rgb', msg)

    def _rear_cb(self, msg: Image) -> None:
        self._push('rear_rgb', msg)

    def _depth_cb(self, msg: Image) -> None:
        self._push('depth', msg)

    def _lidar_scan_cb(self, msg: LaserScan) -> None:
        self._push('lidar_scan', msg)

    def _lidar_points_cb(self, msg: PointCloud2) -> None:
        self._push('lidar_points', msg)

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

    def _save_aligned_snapshot(self) -> None:
        target_ns = self.get_clock().now().nanoseconds
        if self.last_trigger_ns is None:
            # Skip first callback to avoid timer catch-up burst after /clock starts.
            self.last_trigger_ns = target_ns
            return
        if target_ns < self.last_trigger_ns:
            # Sim time reset/backward jump: re-anchor.
            self.last_trigger_ns = target_ns
            return
        if target_ns - self.last_trigger_ns < self.save_interval_ns:
            return
        self.last_trigger_ns = target_ns
        if self.last_saved_target_ns is not None and target_ns == self.last_saved_target_ns:
            return

        tol_ns = int(self.sync_tolerance_sec * 1e9)
        selected: Dict[str, Tuple[int, object, int]] = {}

        for key in ('front_rgb', 'rear_rgb', 'depth'):
            stamp_ns, msg, abs_dt = self._nearest(key, target_ns)
            if stamp_ns is None or msg is None or abs_dt is None:
                self.get_logger().warn(f'Skip snapshot: {key} buffer is empty')
                return
            if abs_dt > tol_ns:
                self.get_logger().warn(
                    f'Skip snapshot: {key} nearest frame too old/new '
                    f'(|dt|={abs_dt / 1e6:.1f}ms, tol={tol_ns / 1e6:.1f}ms)'
                )
                return
            selected[key] = (stamp_ns, msg, abs_dt)

        lidar_scan = self._nearest('lidar_scan', target_ns)
        lidar_points = self._nearest('lidar_points', target_ns)

        chosen_lidar = None
        lidar_kind = None
        for kind, candidate in (('scan', lidar_scan), ('points', lidar_points)):
            stamp_ns, msg, abs_dt = candidate
            if stamp_ns is None or msg is None or abs_dt is None:
                continue
            if abs_dt > tol_ns:
                continue
            if chosen_lidar is None or abs_dt < chosen_lidar[2]:
                chosen_lidar = (stamp_ns, msg, abs_dt)
                lidar_kind = kind

        lidar_available = chosen_lidar is not None
        if not lidar_available and not self.allow_missing_lidar:
            self.get_logger().warn('Skip snapshot: lidar scan/points both unavailable in tolerance window')
            return
        if lidar_available:
            selected['lidar'] = chosen_lidar  # type: ignore[assignment]
        else:
            self.get_logger().warn('Lidar unavailable in tolerance window, saving RGB/Depth only')

        sec = target_ns // 1_000_000_000
        nanosec = target_ns % 1_000_000_000
        frame_name = f'step{self.step_index}'
        frame_dir = os.path.join(self.output_dir, frame_name)
        while os.path.exists(frame_dir):
            self.step_index += 1
            frame_name = f'step{self.step_index}'
            frame_dir = os.path.join(self.output_dir, frame_name)
        os.makedirs(frame_dir, exist_ok=True)

        front_dir = os.path.join(frame_dir, 'front_rgb')
        rear_dir = os.path.join(frame_dir, 'rear_rgb')
        depth_dir = os.path.join(frame_dir, 'depth')
        os.makedirs(front_dir, exist_ok=True)
        os.makedirs(rear_dir, exist_ok=True)
        os.makedirs(depth_dir, exist_ok=True)

        self._save_rgb(selected['front_rgb'][1], os.path.join(front_dir, 'image.png'))
        self._save_rgb(selected['rear_rgb'][1], os.path.join(rear_dir, 'image.png'))
        self._save_depth(selected['depth'][1], os.path.join(depth_dir, 'image.png'))

        lidar_meta = {
            'available': False,
            'msg_time_ns': None,
            'delta_ms': None,
            'path': None,
            'type': None,
            'format': None,
        }
        if lidar_available:
            lidar_dir = os.path.join(frame_dir, 'lidar')
            os.makedirs(lidar_dir, exist_ok=True)
            lidar_path = os.path.join(lidar_dir, 'scan.bin')
            lidar_format = self._save_lidar(selected['lidar'][1], lidar_path, lidar_kind)
            lidar_meta = {
                'available': True,
                'msg_time_ns': selected['lidar'][0],
                'delta_ms': round(selected['lidar'][2] / 1e6, 3),
                'path': 'lidar/scan.bin',
                'type': lidar_kind,
                'format': lidar_format,
            }

        metadata = {
            'step': frame_name,
            'target_time_ns': target_ns,
            'sensors': {
                'front_rgb': {
                    'msg_time_ns': selected['front_rgb'][0],
                    'delta_ms': round(selected['front_rgb'][2] / 1e6, 3),
                    'path': 'front_rgb/image.png',
                },
                'rear_rgb': {
                    'msg_time_ns': selected['rear_rgb'][0],
                    'delta_ms': round(selected['rear_rgb'][2] / 1e6, 3),
                    'path': 'rear_rgb/image.png',
                },
                'depth': {
                    'msg_time_ns': selected['depth'][0],
                    'delta_ms': round(selected['depth'][2] / 1e6, 3),
                    'path': 'depth/image.png',
                },
                'lidar': lidar_meta,
            },
        }
        with open(os.path.join(frame_dir, 'meta.json'), 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2)
        self.last_saved_target_ns = target_ns
        self.step_index += 1

        self.get_logger().info(
            f'Saved aligned frame at {frame_name} ({sec}_{nanosec:09d}) '
            f'(front={metadata["sensors"]["front_rgb"]["delta_ms"]}ms, '
            f'rear={metadata["sensors"]["rear_rgb"]["delta_ms"]}ms, '
            f'depth={metadata["sensors"]["depth"]["delta_ms"]}ms, '
            f'lidar={metadata["sensors"]["lidar"]["delta_ms"] if lidar_available else "N/A"}ms)'
        )

    def _save_rgb(self, msg: Image, path: str) -> None:
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(path, image)

    def _save_depth(self, msg: Image, path: str) -> None:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if depth.dtype == np.float32:
            depth_mm = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
            depth_mm = np.clip(depth_mm, 0, 65535).astype(np.uint16)
        elif depth.dtype == np.uint16:
            depth_mm = depth
        else:
            depth_mm = depth.astype(np.uint16)
        cv2.imwrite(path, depth_mm)

    def _save_lidar(self, msg: object, path: str, lidar_kind: Optional[str]) -> str:
        if lidar_kind == 'scan' and isinstance(msg, LaserScan):
            ranges = np.array(msg.ranges, dtype=np.float32)
            angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
            intensities = np.array(msg.intensities, dtype=np.float32)
            if intensities.size == 0:
                intensities = np.full_like(ranges, np.nan, dtype=np.float32)
            elif intensities.size != ranges.size:
                intensities = np.resize(intensities, ranges.size).astype(np.float32)
            scan = np.stack([angles, ranges, intensities], axis=1).astype(np.float32)
            scan.tofile(path)
            return 'float32 Nx3 [angle, range, intensity]'

        if lidar_kind == 'points' and isinstance(msg, PointCloud2):
            np.frombuffer(msg.data, dtype=np.uint8).tofile(path)
            return (
                f'raw PointCloud2 bytes; point_step={msg.point_step}, row_step={msg.row_step}, '
                f'width={msg.width}, height={msg.height}'
            )

        # Fallback (should not happen)
        np.array([], dtype=np.uint8).tofile(path)
        return 'empty'


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


if __name__ == '__main__':
    main()
