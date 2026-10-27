#!/usr/bin/env python3
"""Re-escala /odom_raw (firmware, r0 y b0 fijos) con wheel_radius y wheel_separation
del YAML y publica /odom + tf odom->base_footprint.
  ds'  = ds  * r/r0
  dth' = dth * r/r0 * b0/b_eff
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class OdomCorrector(Node):
    def __init__(self):
        super().__init__('odom_corrector')
        self.declare_parameter('wheel_radius', 0.0325)             # r  [m]
        self.declare_parameter('wheel_separation', 0.24)           # b_eff [m]
        self.declare_parameter('firmware_wheel_radius', 0.0325)    # r0 [m]
        self.declare_parameter('firmware_wheel_separation', 0.24)  # b0 [m]
        self.declare_parameter('input_topic', '/odom_raw')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self._update_gains()
        self.add_on_set_parameters_callback(self._on_params)

        self.x = 0.0    # pose corregida [m, m, rad]
        self.y = 0.0
        self.th = 0.0
        self.prev = None  # (x, y, yaw) del /odom_raw anterior
        self.n_msgs = 0

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.sub = self.create_subscription(Odometry, in_topic, self._cb, qos_profile_sensor_data)
        self.reset_sub = self.create_subscription(Empty, '/odom_reset', self._reset, 10)
        self.tf_br = TransformBroadcaster(self) if self.get_parameter('publish_tf').value else None
        self.timer = self.create_timer(5.0, self._watchdog)

        self.get_logger().info(
            f'{in_topic} -> {out_topic} | r={self.r:.4f} (r0={self.r0:.4f}) '
            f'b_eff={self.b:.4f} (b0={self.b0:.4f}) k_lin={self.k_lin:.4f} k_ang={self.k_ang:.4f}')

    def _update_gains(self):
        self.r = float(self.get_parameter('wheel_radius').value)
        self.b = float(self.get_parameter('wheel_separation').value)
        self.r0 = float(self.get_parameter('firmware_wheel_radius').value)
        self.b0 = float(self.get_parameter('firmware_wheel_separation').value)
        self.k_lin = self.r / self.r0                    # factor sobre distancia
        self.k_ang = (self.r / self.r0) * (self.b0 / self.b)  # factor sobre giro

    def _on_params(self, params):
        names = {'wheel_radius': 'r', 'wheel_separation': 'b',
                 'firmware_wheel_radius': 'r0', 'firmware_wheel_separation': 'b0'}
        new = {a: getattr(self, a) for a in names.values()}
        for p in params:
            if p.name in names:
                try:
                    v = float(p.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason='debe ser numero')
                if v <= 0.0:
                    return SetParametersResult(successful=False, reason='debe ser > 0')
                new[names[p.name]] = v
        self.r, self.b, self.r0, self.b0 = new['r'], new['b'], new['r0'], new['b0']
        self.k_lin = self.r / self.r0
        self.k_ang = (self.r / self.r0) * (self.b0 / self.b)
        self.get_logger().info(
            f'r={self.r:.4f} b_eff={self.b:.4f} k_lin={self.k_lin:.4f} k_ang={self.k_ang:.4f}')
        return SetParametersResult(successful=True)

    def _reset(self, _msg):
        self.x = self.y = self.th = 0.0
        self.prev = None
        self.get_logger().info('pose en cero')

    def _watchdog(self):
        if self.n_msgs == 0:
            self.get_logger().warn(
                'sin /odom_raw: revisar docker ps (micro_ros_agent), ROS_DOMAIN_ID=20, '
                'o resetear el ESP32 (sh ~/reset_esp32.sh en la Pi)')
        self.n_msgs = 0

    def _cb(self, msg):
        self.n_msgs += 1
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pyaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.prev is not None:
            x0, y0, yaw0 = self.prev
            dx, dy = px - x0, py - y0
            ds = math.hypot(dx, dy)
            if dx * math.cos(yaw0) + dy * math.sin(yaw0) < 0.0:
                ds = -ds
            dth = wrap(pyaw - yaw0)

            ds *= self.k_lin
            dth *= self.k_ang
            th_mid = self.th + 0.5 * dth
            self.x += ds * math.cos(th_mid)
            self.y += ds * math.sin(th_mid)
            self.th = wrap(self.th + dth)
        self.prev = (px, py, pyaw)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.orientation = quat_from_yaw(self.th)
        out.pose.covariance = msg.pose.covariance
        out.twist.twist.linear.x = msg.twist.twist.linear.x * self.k_lin
        out.twist.twist.linear.y = msg.twist.twist.linear.y * self.k_lin
        out.twist.twist.angular.z = msg.twist.twist.angular.z * self.k_ang
        out.twist.covariance = msg.twist.covariance
        self.pub.publish(out)

        if self.tf_br is not None:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = out.pose.pose.orientation
            self.tf_br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
