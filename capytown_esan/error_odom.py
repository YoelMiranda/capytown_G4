#!/usr/bin/env python3
"""Lee /odom de un ros2 bag, grafica la trayectoria contra el cuadrado ideal y
calcula el error de cierre.  Uso: error_odom <bag> [lado_m]"""
import sys
import math

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

from capytown_esan.bag_utils import storage_id

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_odom(bag_path, odom_topic='/odom'):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id=storage_id(bag_path)),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'),
    )
    xs, ys = [], []
    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(data, Odometry)
            xs.append(msg.pose.pose.position.x)
            ys.append(msg.pose.pose.position.y)
    return xs, ys


def ideal_square(side=1.0):
    return [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]


def plot(xs, ys, out_png, side=1.0):
    if not xs:
        print('[ERROR] No se encontraron mensajes en /odom dentro del bag.')
        return None

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(xs, ys, label='trayectoria /odom', color='#B85042', linewidth=1.5)
    ideal = ideal_square(side)
    ax.plot([p[0] for p in ideal], [p[1] for p in ideal],
            '--', label='cuadrado ideal', color='#5C6D3A')
    ax.scatter([xs[0]], [ys[0]], c='green', s=60, zorder=5, label='inicio')
    ax.scatter([xs[-1]], [ys[-1]], c='red', s=60, zorder=5, label='fin')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Trayectoria /odom')

    dx = xs[-1] - xs[0]
    dy = ys[-1] - ys[0]
    err = math.hypot(dx, dy)
    ax.annotate(f'error cierre = {err*100:.1f} cm',
                xy=(xs[-1], ys[-1]), xytext=(0.05, 0.95),
                textcoords='axes fraction', fontsize=10,
                bbox=dict(boxstyle='round', fc='wheat', alpha=0.7))

    fig.savefig(out_png, dpi=150, bbox_inches='tight')
    print(f'Grafica guardada en: {out_png}')
    print(f'Error de cierre: dx={dx*100:+.2f} cm, dy={dy*100:+.2f} cm, total={err*100:.2f} cm')
    return err


def main():
    if len(sys.argv) < 2:
        print('Uso: ros2 run capytown_esan error_odom <ruta_al_bag> [lado_m]')
        return
    bag_path = sys.argv[1]
    side = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    xs, ys = read_odom(bag_path)
    out_png = bag_path.rstrip('/') + '_trayectoria.png'
    plot(xs, ys, out_png, side)


if __name__ == '__main__':
    main()
