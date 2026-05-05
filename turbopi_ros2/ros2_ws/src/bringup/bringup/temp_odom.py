#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class TempOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_odometry_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("use_imu_yaw_rate", True)
        self.declare_parameter("yaw_rate_scale", 1.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("linear_scale_x", 1.0)
        self.declare_parameter("linear_scale_y", 1.0)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.use_imu_yaw_rate = bool(self.get_parameter("use_imu_yaw_rate").value)
        self.yaw_rate_scale = float(self.get_parameter("yaw_rate_scale").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.linear_scale_x = float(self.get_parameter("linear_scale_x").value)
        self.linear_scale_y = float(self.get_parameter("linear_scale_y").value)

        self.vx_body = 0.0
        self.vy_body = 0.0
        self.wz_cmd = 0.0
        self.wz_imu = 0.0
        self.imu_yaw = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        now = self.get_clock().now()
        self.last_update_time = now
        self.last_cmd_time = now

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10,
        )
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.update)

        self.get_logger().info(
            f"Fake odometry node started. "
            f"cmd_vel={self.cmd_vel_topic}, imu={self.imu_topic}, odom={self.odom_topic}"
        )

    def cmd_callback(self, msg: Twist) -> None:
        self.vx_body = float(msg.linear.x) * self.linear_scale_x
        self.vy_body = float(msg.linear.y) * self.linear_scale_y
        self.wz_cmd = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def imu_callback(self, msg: Imu) -> None:
        self.wz_imu = float(msg.angular_velocity.z) * self.yaw_rate_scale
        q = msg.orientation
        self.imu_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_update_time = now

        age_since_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age_since_cmd > self.cmd_timeout_s:
            vx_body = 0.0
            vy_body = 0.0
            wz_cmd = 0.0
        else:
            vx_body = self.vx_body
            vy_body = self.vy_body
            wz_cmd = self.wz_cmd

        wz = self.wz_imu if self.use_imu_yaw_rate else wz_cmd

        if self.use_imu_yaw_rate:
            self.yaw = self.imu_yaw
        else:
            self.yaw += wz_cmd * dt
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        vx_world = vx_body * cos_yaw - vy_body * sin_yaw
        vy_world = vx_body * sin_yaw + vy_body * cos_yaw

        self.x += vx_world * dt
        self.y += vy_world * dt

        self.publish_odom(now, vx_body, vy_body, wz)
        self.publish_tf(now)

    def publish_odom(self, now, vx_body: float, vy_body: float, wz: float) -> None:
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = vx_body
        msg.twist.twist.linear.y = vy_body
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = wz

        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]
        msg.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]

        self.odom_pub.publish(msg)

    def publish_tf(self, now) -> None:
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TempOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()