#!/usr/bin/env python3
"""Cuadrado en lazo abierto (por tiempo): avanza `side` m y gira 90 grados, 4 lados x
`laps` vueltas; graba /odom y al final grafica trayectoria vs ideal + error de cierre.
Tecla q = parada de emergencia."""
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import matplotlib
if not os.environ.get('DISPLAY'):
    matplotlib.use('Agg')
import matplotlib.pyplot as plt


class SquareRunner(Node):
    def __init__(self):
        super().__init__('run_square')
        self.declare_parameter('side', 1.0)          # lado [m]
        self.declare_parameter('v_lin', 0.10)        # [m/s]
        self.declare_parameter('v_ang', 0.50)        # [rad/s]
        self.declare_parameter('turn_dir', 1.0)      # 1.0 antihorario, -1.0 horario
        self.declare_parameter('laps', 3)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('brake_s', 0.4)       # pausa entre tramos [s]
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('out_png', 'trayectoria_cuadrado.png')

        self.side = float(self.get_parameter('side').value)
        self.v_lin = float(self.get_parameter('v_lin').value)
        self.v_ang = float(self.get_parameter('v_ang').value)
        self.turn_dir = float(self.get_parameter('turn_dir').value)
        self.laps = int(self.get_parameter('laps').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.brake_s = float(self.get_parameter('brake_s').value)
        self.out_png = self.get_parameter('out_png').value
        odom_topic = self.get_parameter('odom_topic').value

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 50)
        self.xs, self.ys = [], []
        self.abort = False

    def _odom_cb(self, msg):
        self.xs.append(msg.pose.pose.position.x)
        self.ys.append(msg.pose.pose.position.y)

    def _check_q(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr and sys.stdin.read(1) == 'q':
            self.abort = True
            self.get_logger().warn('>>> PARADA DE EMERGENCIA (q) <<<')

    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def _drive(self, vx, wz, duration):
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        period = 1.0 / self.rate
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if self.abort:
                return
            self._check_q()
            if self.abort:
                return
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def run_square(self):
        t_fwd = self.side / self.v_lin
        t_turn = (math.pi / 2.0) / self.v_ang
        self.get_logger().info(
            f'Cuadrado {self.side}x{self.side} m, {self.laps} vueltas. '
            f'Recto {t_fwd:.1f}s, giro {t_turn:.2f}s. Pulsa q para parar.')
        for lap in range(self.laps):
            if self.abort:
                break
            self.get_logger().info(f'--- vuelta {lap + 1}/{self.laps} ---')
            for _side in range(4):
                if self.abort:
                    break
                self._drive(self.v_lin, 0.0, t_fwd)
                self._drive(0.0, 0.0, self.brake_s)
                self._drive(0.0, self.v_ang * self.turn_dir, t_turn)
                self._drive(0.0, 0.0, self.brake_s)
        self.stop()

    def plot(self):
        if not self.xs:
            self.get_logger().error(
                'No se grabo odometria. Prueba con -p odom_topic:=/odom_raw o revisa ros2 topic list.')
            return
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(self.xs, self.ys, '-', color='#B85042', lw=1.5, label='trayectoria /odom')
        s = self.side
        ideal = [(0, 0), (s, 0), (s, s), (0, s), (0, 0)]
        ax.plot([p[0] for p in ideal], [p[1] for p in ideal], '--',
                color='#5C6D3A', label='cuadrado ideal')
        ax.scatter([self.xs[0]], [self.ys[0]], c='green', s=60, zorder=5, label='inicio')
        ax.scatter([self.xs[-1]], [self.ys[-1]], c='red', s=60, zorder=5, label='fin')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('Trayectoria' + ('  (PARADA DE EMERGENCIA)' if self.abort else ''))

        dx = self.xs[-1] - self.xs[0]
        dy = self.ys[-1] - self.ys[0]
        err = math.hypot(dx, dy)
        ax.annotate(f'error de cierre (odom) = {err * 100:.1f} cm',
                    xy=(0.05, 0.95), xycoords='axes fraction', fontsize=10,
                    bbox=dict(boxstyle='round', fc='wheat', alpha=0.8))

        out = os.path.abspath(self.out_png)
        fig.savefig(out, dpi=150, bbox_inches='tight')
        print('\n' + '=' * 56)
        print(f' Imagen guardada en: {out}')
        print(f' Error de cierre (segun /odom): dx={dx * 100:+.1f} cm, '
              f'dy={dy * 100:+.1f} cm, total={err * 100:.1f} cm')
        print('=' * 56)

        if os.environ.get('DISPLAY'):
            try:
                plt.show()
            except Exception as e:
                print(f'(No se pudo abrir ventana: {e}. Revisa el PNG guardado.)')


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = SquareRunner()
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.run_square()
    except KeyboardInterrupt:
        node.abort = True
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
