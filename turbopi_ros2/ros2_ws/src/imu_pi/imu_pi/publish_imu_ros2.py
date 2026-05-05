#!/usr/bin/env python3
"""Publish BNO055 IMU data to ROS2 on /imu/data."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from imu_pi.imu_tracking_common import build_tracking_runtime, select_tracking_physical_fields


class Bno055TrackingPublisher(Node):
    """ROS2 node that publishes BNO055 IMU data on /imu/data."""

    def __init__(self) -> None:
        super().__init__("bno055_tracking_publisher")
        self._runtime = build_tracking_runtime()

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(Imu, "/imu/data", qos)
        self._timer = self.create_timer(self._runtime.period_s, self._publish_sample)

        self.get_logger().info(
            f"Publishing IMU data on /imu/data "
            f"from bus={self._runtime.config.bus} "
            f"address=0x{self._runtime.config.address:02X} "
            f"at {self._runtime.config.poll_hz:.1f} Hz"
        )

    def _publish_sample(self) -> None:
        """Read one sensor sample and publish it."""

        try:
            raw_sample = self._runtime.reader.read_raw()
            tracking_sample = select_tracking_physical_fields(raw_sample)

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            quat = self._runtime.reader.read_quaternion()
            msg.orientation.x = quat.x
            msg.orientation.y = quat.y
            msg.orientation.z = quat.z
            msg.orientation.w = quat.w

            msg.angular_velocity.x = tracking_sample.angular_velocity_rad_s.x
            msg.angular_velocity.y = tracking_sample.angular_velocity_rad_s.y
            msg.angular_velocity.z = tracking_sample.angular_velocity_rad_s.z

            msg.linear_acceleration.x = tracking_sample.linear_acceleration_m_s2.x
            msg.linear_acceleration.y = tracking_sample.linear_acceleration_m_s2.y
            msg.linear_acceleration.z = tracking_sample.linear_acceleration_m_s2.z

            accel_variance = 0.05
            gyro_variance = 0.02

            msg.linear_acceleration_covariance = [
                accel_variance, 0.0, 0.0,
                0.0, accel_variance, 0.0,
                0.0, 0.0, accel_variance,
            ]
            msg.angular_velocity_covariance = [
                gyro_variance, 0.0, 0.0,
                0.0, gyro_variance, 0.0,
                0.0, 0.0, gyro_variance,
            ]

            self._publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to read or publish IMU sample: {exc}")


def main() -> None:
    """Initialize ROS2 and spin the publisher until the node is stopped."""

    rclpy.init()
    node = Bno055TrackingPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
