#!/usr/bin/env python3
"""Simulador sin hardware: suscribe /cmd_vel, publica /odom (integra omega comandada),
/odom_true y /true_yaw_deg (pose fisica con omega * b_model / b_true) y tf odom->base_link."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.declare_parameter('wheel_separation', 0.24)   # b_eff que usa la odometria [m]
        self.declare_parameter('b_eff_true', 0.27)          # b_eff fisico real [m]
        self.declare_parameter('rate_hz', 50.0)

        self.b_model = float(self.get_parameter('wheel_separation').value)
        self.b_true = float(self.get_parameter('b_eff_true').value)
        self.rate = float(self.get_parameter('rate_hz').value)

        self.x_o = self.y_o = self.yaw_o = 0.0   # pose segun odometria
        self.x_t = self.y_t = self.yaw_t = 0.0   # pose fisica real
        self.v = 0.0        # [m/s] comandada
        self.w_cmd = 0.0    # [rad/s] comandada

        self.sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_true = self.create_publisher(Odometry, '/odom_true', 20)
        self.pub_yaw = self.create_publisher(Float64, '/true_yaw_deg', 20)
        self.tf_bc = TransformBroadcaster(self)

        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self._step)
        self.get_logger().info(
            f'fake_robot: b_eff_model={self.b_model:.4f}  b_eff_true={self.b_true:.4f}  '
            f'(ratio fisico/odom = {self.b_model / self.b_true:.4f})')

    def _cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w_cmd = msg.angular.z

    def _step(self):
        dt = self.dt
        self.yaw_o += self.w_cmd * dt
        self.x_o += self.v * math.cos(self.yaw_o) * dt
        self.y_o += self.v * math.sin(self.yaw_o) * dt
        w_phys = self.w_cmd * (self.b_model / self.b_true)
        self.yaw_t += w_phys * dt
        self.x_t += self.v * math.cos(self.yaw_t) * dt
        self.y_t += self.v * math.sin(self.yaw_t) * dt

        now = self.get_clock().now().to_msg()
        self._publish(self.pub_odom, now, self.x_o, self.y_o, self.yaw_o)
        self._publish(self.pub_true, now, self.x_t, self.y_t, self.yaw_t)
        self.pub_yaw.publish(Float64(data=math.degrees(self.yaw_t)))

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x_o
        t.transform.translation.y = self.y_o
        qx, qy, qz, qw = quat_from_yaw(self.yaw_o)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_bc.sendTransform(t)

    @staticmethod
    def _publish(pub, stamp, x, y, yaw):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        qx, qy, qz, qw = quat_from_yaw(yaw)
        o.pose.pose.orientation.x = qx
        o.pose.pose.orientation.y = qy
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        pub.publish(o)


def main():
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
