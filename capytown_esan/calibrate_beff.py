#!/usr/bin/env python3
"""Gira en sitio hasta que /odom acumule target_angle_deg; con el angulo fisico medido
calcula b_eff_nuevo = b_eff * objetivo / medido y lo registra en calibration_log.csv."""
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class BeffCalibrator(Node):
    def __init__(self):
        super().__init__('calibrate_beff')
        self.declare_parameter('b_eff', 0.24)              # b_eff actual del YAML [m]
        self.declare_parameter('target_angle_deg', 360.0)  # giro objetivo segun /odom [deg]
        self.declare_parameter('angular_vel', 0.5)         # [rad/s], + = antihorario
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')

        self.b_eff = float(self.get_parameter('b_eff').value)
        self.target_deg = float(self.get_parameter('target_angle_deg').value)
        self.ang_vel = float(self.get_parameter('angular_vel').value)
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.cur_yaw = None

    def _odom_cb(self, msg):
        self.cur_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def wait_for_odom(self, timeout=10.0):
        t0 = self.get_clock().now()
        while rclpy.ok() and self.cur_yaw is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > timeout:
                return False
        return True

    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def rotate_until_target(self):
        target_rad = math.radians(self.target_deg)
        accumulated = 0.0
        last_yaw = self.cur_yaw
        tw = Twist()
        tw.angular.z = self.ang_vel
        max_time = 3.0 * (target_rad / max(self.ang_vel, 1e-3))
        t0 = self.get_clock().now()
        self.get_logger().info(
            f'Girando a {self.ang_vel:.2f} rad/s hasta que /odom marque {self.target_deg:.0f} grados...')
        rate_hz = 20.0
        while rclpy.ok() and accumulated < target_rad:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)
            if self.cur_yaw is not None and last_yaw is not None:
                accumulated += abs(ang_diff(self.cur_yaw, last_yaw))
            last_yaw = self.cur_yaw
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > max_time:
                self.get_logger().warn(
                    'TIMEOUT girando. El robot no rota o /odom no avanza. '
                    'Revisa: ROS_DOMAIN_ID, agente micro-ROS, y que /cmd_vel mueva el robot.')
                break
        self.stop()
        return math.degrees(accumulated)


def main():
    rclpy.init()
    node = BeffCalibrator()

    log_path = os.path.join(os.getcwd(), 'calibration_log.csv')
    is_new = not os.path.exists(log_path)
    log_file = open(log_path, 'a', newline='')
    writer = csv.writer(log_file)
    if is_new:
        writer.writerow(['timestamp', 'iteracion', 'b_eff_antes', 'objetivo_deg',
                         'medido_deg', 'b_eff_despues', 'error_pct'])

    print('=' * 64)
    print(' CALIBRACION DE b_eff')
    print('=' * 64)
    print(f' b_eff actual (debe coincidir con wheel_params.yaml): {node.b_eff:.4f} m')
    print(f' Objetivo de giro: {node.target_deg:.0f} grados a {node.ang_vel:.2f} rad/s')
    print(' >> Marca una linea de referencia en el robot Y en el piso antes de girar.')
    print('-' * 64)

    if not node.wait_for_odom():
        print('\n[ERROR] No llega /odom:')
        print('  1) export ROS_DOMAIN_ID=20 en ESTA terminal?')
        print('  2) bringup lanzado (ros2 launch capytown_esan bringup.launch.py)?')
        print('  3) ros2 topic hz /odom_raw publica? Si no: sh ~/reset_esp32.sh en la Pi')
        node.destroy_node()
        rclpy.shutdown()
        log_file.close()
        return

    iteration = 0
    try:
        input('\nENTER para iniciar el giro de calibracion (Ctrl+C para salir)... ')
        iteration += 1

        odom_deg = node.rotate_until_target()
        print(f'\n  /odom acumulo ~{odom_deg:.1f} grados (objetivo {node.target_deg:.0f}).')
        print('  Ahora MIDE con el transportador cuanto giro FISICAMENTE el robot.')

        medido = float(input('  Angulo fisico real medido (grados): ').strip())
        if medido <= 0:
            print('  Valor invalido.')
            raise SystemExit

        b_before = node.b_eff
        b_after = b_before * (node.target_deg / medido)
        error_pct = abs(medido - node.target_deg) / node.target_deg * 100.0

        print('\n  ' + '-' * 58)
        print(f'  b_eff corregido = {b_before:.4f} x ({node.target_deg:.0f} / {medido:.1f})'
              f' = {b_after:.4f} m')
        if medido < node.target_deg:
            print('  -> Giro de MENOS: b_eff era pequenho. AUMENTA b_eff.')
        elif medido > node.target_deg:
            print('  -> Giro de MAS: b_eff era grande. REDUCE b_eff.')
        else:
            print('  -> Giro exacto.')
        print(f'  Error actual de giro: {error_pct:.2f} %')
        if error_pct < 1.0:
            print('  [OK] Menos de 1% -> CONVERGIO. Adopta este b_eff.')
        print('  ' + '-' * 58)

        writer.writerow([datetime.now().isoformat(timespec='seconds'), iteration,
                         f'{b_before:.4f}', f'{node.target_deg:.0f}', f'{medido:.1f}',
                         f'{b_after:.4f}', f'{error_pct:.2f}'])
        log_file.flush()

        print('\n  SIGUIENTE PASO:')
        print(f'   1) config/wheel_params.yaml -> wheel_separation: {b_after:.4f}')
        print('   2) Relanza el bringup.')
        print(f'   3) Vuelve a correr este script con  -p b_eff:={b_after:.4f}')
        print('   Repite hasta que el error sea < 1 %.')

    except (KeyboardInterrupt, EOFError, SystemExit):
        pass
    finally:
        node.stop()
        log_file.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f'\nLog de calibracion: {log_path}')


if __name__ == '__main__':
    main()
