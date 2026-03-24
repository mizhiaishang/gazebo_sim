#!/usr/bin/env python3
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


class DepthLidarFallback(Node):
    def __init__(self) -> None:
        super().__init__("depth_lidar_fallback")
        self.bridge = CvBridge()

        self.depth_topic = self.declare_parameter(
            "depth_topic", "/agent/depth_camera/image_raw"
        ).get_parameter_value().string_value
        self.scan_topic = self.declare_parameter(
            "scan_topic", "/agent/lidar/scan"
        ).get_parameter_value().string_value
        self.horizontal_fov_rad = float(
            self.declare_parameter("horizontal_fov_rad", 1.396).get_parameter_value().double_value
        )
        self.sample_count = int(self.declare_parameter("sample_count", 360).get_parameter_value().integer_value)
        self.row_ratio = float(self.declare_parameter("row_ratio", 0.5).get_parameter_value().double_value)
        self.range_min = float(self.declare_parameter("range_min", 0.12).get_parameter_value().double_value)
        self.range_max = float(self.declare_parameter("range_max", 15.0).get_parameter_value().double_value)
        self.publish_rate_hz = max(
            1.0, float(self.declare_parameter("publish_rate_hz", 10.0).get_parameter_value().double_value)
        )
        self.publish_period_ns = int(1e9 / self.publish_rate_hz)
        self.last_pub_ns = 0

        self.pub = self.create_publisher(LaserScan, self.scan_topic, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, qos_profile_sensor_data)

        self.get_logger().warn(
            "Depth lidar fallback enabled. "
            f"depth_topic={self.depth_topic}, scan_topic={self.scan_topic}, "
            f"fov={self.horizontal_fov_rad:.3f}rad, samples={self.sample_count}"
        )

    def _stamp_ns(self, msg: Image) -> int:
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)
        if sec == 0 and nsec == 0:
            return self.get_clock().now().nanoseconds
        return sec * 1_000_000_000 + nsec

    def _to_depth_m(self, depth: np.ndarray) -> np.ndarray:
        if depth.dtype == np.float32:
            depth_m = depth
        elif depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)
        return np.nan_to_num(depth_m, nan=np.inf, posinf=np.inf, neginf=np.inf)

    def _depth_cb(self, msg: Image) -> None:
        stamp_ns = self._stamp_ns(msg)
        if self.last_pub_ns != 0 and stamp_ns - self.last_pub_ns < self.publish_period_ns:
            return

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth.ndim > 2:
            depth = depth[:, :, 0]
        depth_m = self._to_depth_m(depth)
        h, w = depth_m.shape[:2]
        if h <= 0 or w <= 1:
            return

        row = int(np.clip(round(self.row_ratio * (h - 1)), 0, h - 1))
        line = depth_m[row, :]
        sample_count = int(np.clip(self.sample_count, 2, w))
        idx = np.linspace(0, w - 1, num=sample_count, dtype=np.int32)
        ranges = line[idx].astype(np.float32)

        invalid = (ranges < self.range_min) | (ranges > self.range_max)
        ranges[invalid] = np.inf

        scan = LaserScan()
        scan.header = msg.header
        if not scan.header.frame_id:
            scan.header.frame_id = "lidar_link"
        scan.angle_min = -0.5 * self.horizontal_fov_rad
        scan.angle_max = 0.5 * self.horizontal_fov_rad
        scan.angle_increment = (scan.angle_max - scan.angle_min) / float(sample_count - 1)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.publish_rate_hz
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        scan.intensities = []

        self.pub.publish(scan)
        self.last_pub_ns = stamp_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthLidarFallback()
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
