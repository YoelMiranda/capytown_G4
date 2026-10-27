#!/usr/bin/env python3
"""Utilidades para leer bags de ROS 2 (sqlite3 o mcap) y calcular metricas de la
recta y del giro. Las funciones de metricas son puras (sin ROS) y se prueban con
pytest; rosbag2_py y rclpy se importan solo dentro de read_pose()."""
import math
import os

import yaml


def _bag_dir(bag_path):
    """Acepta la carpeta del bag o la ruta a su .db3/.mcap."""
    bag_path = os.path.normpath(bag_path)
    if os.path.isfile(bag_path):
        return os.path.dirname(bag_path)
    return bag_path


def _metadata(bag_path):
    meta_path = os.path.join(_bag_dir(bag_path), 'metadata.yaml')
    if not os.path.isfile(meta_path):
        raise FileNotFoundError(
            f'No existe {meta_path}. Pasa la carpeta del bag (la que crea '
            f'`ros2 bag record -o <nombre>`); si el record se corto con kill -9 '
            f'el bag puede no tener metadata.yaml.')
    with open(meta_path) as f:
        return yaml.safe_load(f)['rosbag2_bagfile_information']


def storage_id(bag_path):
    """Formato de almacenamiento del bag ('sqlite3' o 'mcap') segun metadata.yaml."""
    return _metadata(bag_path)['storage_identifier']


def topic_counts(bag_path):
    """{topico: n_mensajes} segun metadata.yaml (sirve para avisar si falta un topico)."""
    out = {}
    for item in _metadata(bag_path).get('topics_with_message_count', []) or []:
        out[item['topic_metadata']['name']] = int(item['message_count'])
    return out


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_pose(bag_path, topic='/odom'):
    """Lee nav_msgs/Odometry de `topic`. Devuelve listas t [s, relativo al primer
    mensaje], x [m], y [m], theta [rad, en (-pi, pi]]."""
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    bag_dir = _bag_dir(bag_path)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id=storage_id(bag_dir)),
                ConverterOptions('cdr', 'cdr'))
    try:
        # el filtro solo acelera la lectura; abajo se filtra igual por topico
        from rosbag2_py import StorageFilter
        reader.set_filter(StorageFilter(topics=[topic]))
    except (ImportError, AttributeError, RuntimeError):
        pass

    t, x, y, th = [], [], [], []
    t0 = None
    while reader.has_next():
        tp, data, stamp_ns = reader.read_next()
        if tp != topic:
            continue
        msg = deserialize_message(data, Odometry)
        if t0 is None:
            t0 = stamp_ns
        t.append((stamp_ns - t0) * 1e-9)
        x.append(msg.pose.pose.position.x)
        y.append(msg.pose.pose.position.y)
        th.append(yaw_from_quat(msg.pose.pose.orientation))
    return t, x, y, th


# ---------------------------------------------------------------- funciones puras

def unwrap(thetas):
    """Desenrolla una serie de angulos [rad] quitando los saltos de +-2*pi."""
    out = []
    offset = 0.0
    prev = None
    for a in thetas:
        if prev is not None:
            d = a - prev
            if d > math.pi:
                offset -= 2.0 * math.pi
            elif d < -math.pi:
                offset += 2.0 * math.pi
        out.append(a + offset)
        prev = a
    return out


def metricas_recta(x, y, th):
    """Metricas de una recta respecto a la pose inicial:
      d_odom         distancia euclidiana primer -> ultimo punto [m]
      avance         proyeccion del desplazamiento sobre el heading inicial [m]
      desvio_lateral componente perpendicular al heading inicial [m] (+ = izquierda)
      dtheta_deg     cambio de orientacion acumulado [deg]"""
    if len(x) < 2:
        raise ValueError('se necesitan al menos 2 poses para medir la recta')
    dx = x[-1] - x[0]
    dy = y[-1] - y[0]
    h0 = th[0]
    u = unwrap(th)
    return {
        'd_odom': math.hypot(dx, dy),
        'avance': dx * math.cos(h0) + dy * math.sin(h0),
        'desvio_lateral': -dx * math.sin(h0) + dy * math.cos(h0),
        'dtheta_deg': math.degrees(u[-1] - u[0]),
    }


def metricas_giro(th):
    """Giro acumulado [deg] con signo (+ = antihorario) a partir de theta desenrollado."""
    if len(th) < 2:
        raise ValueError('se necesitan al menos 2 poses para medir el giro')
    u = unwrap(th)
    return math.degrees(u[-1] - u[0])
