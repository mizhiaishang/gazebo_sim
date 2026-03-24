#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


class SensorMonitor(Node):
    def __init__(self) -> None:
        super().__init__('sensor_monitor')

        self.last_seen = {
            'front_rgb': None,
            'rear_rgb': None,
            'depth': None,
            'lidar_scan': None,
        }

        self.create_subscription(Image, '/agent/front_camera/image_raw', self._front_rgb_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/agent/rear_camera/image_raw', self._rear_rgb_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/agent/depth_camera/image_raw', self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/agent/lidar/scan', self._lidar_scan_cb, qos_profile_sensor_data)

        self.create_timer(2.0, self._report_status)
        self.get_logger().info('Sensor monitor started.')

    def _front_rgb_cb(self, _msg: Image) -> None:
        self.last_seen['front_rgb'] = self.get_clock().now()

    def _rear_rgb_cb(self, _msg: Image) -> None:
        self.last_seen['rear_rgb'] = self.get_clock().now()

    def _depth_cb(self, _msg: Image) -> None:
        self.last_seen['depth'] = self.get_clock().now()

    def _lidar_scan_cb(self, _msg: LaserScan) -> None:
        self.last_seen['lidar_scan'] = self.get_clock().now()

    def _report_status(self) -> None:
        now = self.get_clock().now()
        missing = []
        stale = []

        for name, stamp in self.last_seen.items():
            if stamp is None:
                missing.append(name)
                continue
            age_sec = (now - stamp).nanoseconds / 1e9
            if age_sec > 3.0:
                stale.append(f'{name}({age_sec:.1f}s)')

        if missing or stale:
            self.get_logger().warn(
                f'Sensor status issue | missing={missing if missing else "none"} '
                f'| stale={stale if stale else "none"}'
            )
        else:
            self.get_logger().info('All sensor topics are active.')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorMonitor()
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
