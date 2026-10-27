#!/usr/bin/env python3
"""Gira en sitio hasta que /odom acumule target_angle_deg; con el angulo fisico medido
calcula b_eff_nuevo = b_eff * objetivo / medido y lo registra en calibration_log.csv.

angular_vel > 0 gira antihorario y angular_vel < 0 gira horario; el angulo medido con
transportador se ingresa SIEMPRE positivo (magnitud del giro).

Parametros extra:
  modo      'calibracion' (defecto): calcula y sugiere b_eff, escribe calibration_log.csv
            'validacion': no recalibra; escribe variante, prueba, valor_real, valor_odom,
                          error_% en validacion.csv
  variante  'A1' por defecto (se registra en validacion.csv)
  log_file  CSV de salida (vacio = calibration_log.csv o validacion.csv segun modo)
  forzar    true para seguir aunque b_eff no coincida con wheel_separation de /odom_corrector"""
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters


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
        self.declare_parameter('modo', 'calibracion')      # 'calibracion' | 'validacion'
        self.declare_parameter('variante', 'A1')
        self.declare_parameter('log_file', '')             # vacio = segun modo
        self.declare_parameter('forzar', False)

        self.b_eff = float(self.get_parameter('b_eff').value)
        self.target_deg = float(self.get_parameter('target_angle_deg').value)
        self.ang_vel = float(self.get_parameter('angular_vel').value)
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.modo = str(self.get_parameter('modo').value).strip().lower()
        self.variante = str(self.get_parameter('variante').value).strip()
        self.log_file = str(self.get_parameter('log_file').value).strip()
        self.forzar = bool(self.get_parameter('forzar').value)

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

    def read_corrector_params(self, timeout=3.0):
        """(wheel_separation, wheel_radius) activos en /odom_corrector, o None si no responde."""
        cli = self.create_client(GetParameters, '/odom_corrector/get_parameters')
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            req = GetParameters.Request()
            req.names = ['wheel_separation', 'wheel_radius']
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            if not fut.done() or fut.result() is None or len(fut.result().values) != 2:
                return None
            vals = []
            for v in fut.result().values:
                if v.type == ParameterType.PARAMETER_DOUBLE:
                    vals.append(v.double_value)
                elif v.type == ParameterType.PARAMETER_INTEGER:
                    vals.append(float(v.integer_value))
                else:
                    return None
            return vals[0], vals[1]
        finally:
            self.destroy_client(cli)

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
        # abs(): con angular_vel < 0 (horario) el timeout seria de horas
        max_time = 3.0 * (target_rad / max(abs(self.ang_vel), 1e-3))
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


CAMPOS_CALIBRACION = ['timestamp', 'iteracion', 'b_eff_antes', 'objetivo_deg',
                      'medido_deg', 'b_eff_despues', 'error_pct', 'odom_deg']
CAMPOS_VALIDACION = ['variante', 'prueba', 'valor_real', 'valor_odom', 'error_%']


def append_csv(path, header, row):
    """Anexa una fila; escribe el encabezado si el archivo no existe o esta vacio."""
    is_new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        writer = csv.writer(f)
        if is_new:
            writer.writerow(header)
        writer.writerow(row)


def check_corrector(node):
    """Compara -p b_eff con wheel_separation activo en /odom_corrector. False = abortar."""
    activos = node.read_corrector_params()
    if activos is None:
        print(' [AVISO] /odom_corrector/get_parameters no responde (3 s): no se puede comprobar')
        print('         que b_eff coincida con wheel_separation. Se continua.')
        return True
    b_activo, r_activo = activos
    print(f' /odom_corrector activo: wheel_separation = {b_activo:.4f} m, '
          f'wheel_radius (r) = {r_activo:.4f} m')
    if abs(node.b_eff - b_activo) <= 1e-4:
        return True
    print(f'\n[ERROR] -p b_eff:={node.b_eff:.4f} no coincide con wheel_separation = '
          f'{b_activo:.4f} de /odom_corrector.')
    print('  El calculo usaria un b_eff que la odometria no esta usando.')
    print(f'  Corre de nuevo con  -p b_eff:={b_activo:.4f}  o relanza el bringup con el YAML correcto.')
    if node.forzar:
        print('  forzar:=true -> se continua.')
        return True
    print('  (Para seguir de todos modos: -p forzar:=true)')
    return False


def main():
    rclpy.init()
    node = BeffCalibrator()

    if node.modo not in ('calibracion', 'validacion'):
        print(f"[ERROR] modo '{node.modo}' invalido: usa calibracion o validacion.")
        node.destroy_node()
        rclpy.shutdown()
        return
    validacion = node.modo == 'validacion'
    default_log = 'validacion.csv' if validacion else 'calibration_log.csv'
    log_path = os.path.join(os.getcwd(), node.log_file or default_log)
    sentido = 'antihorario' if node.ang_vel > 0 else 'horario'

    print('=' * 64)
    print(' VALIDACION DE GIRO (sin recalibrar)' if validacion else ' CALIBRACION DE b_eff')
    print('=' * 64)
    print(f' Modo: {node.modo}   Variante: {node.variante}   Registro: {log_path}')
    print(f' b_eff actual (debe coincidir con wheel_params.yaml): {node.b_eff:.4f} m')
    print(f' Objetivo de giro: {node.target_deg:.0f} grados a {node.ang_vel:.2f} rad/s ({sentido})')
    print(' >> Marca una linea de referencia en el robot Y en el piso antes de girar.')
    print('-' * 64)

    if not node.wait_for_odom():
        print('\n[ERROR] No llega /odom:')
        print('  1) export ROS_DOMAIN_ID=20 en ESTA terminal?')
        print('  2) bringup lanzado (ros2 launch capytown_esan bringup.launch.py)?')
        print('  3) ros2 topic hz /odom_raw publica? Si no: sh ~/reset_esp32.sh en la Pi')
        node.destroy_node()
        rclpy.shutdown()
        return

    if not check_corrector(node):
        node.destroy_node()
        rclpy.shutdown()
        return
    print('-' * 64)

    iteration = 0
    try:
        input(f'\nENTER para iniciar el giro de {node.modo} (Ctrl+C para salir)... ')
        iteration += 1

        odom_deg = node.rotate_until_target()
        print(f'\n  /odom acumulo ~{odom_deg:.1f} grados (objetivo {node.target_deg:.0f}).')
        print('  Ahora MIDE con el transportador cuanto giro FISICAMENTE el robot.')

        try:
            medido = float(input('  Angulo fisico real medido (grados, siempre positivo): ').strip())
        except ValueError:
            medido = -1.0
        if medido <= 0:
            print('  Valor invalido.')
            raise SystemExit

        if validacion:
            # sin recalibrar: no se calcula ni se sugiere un b_eff nuevo
            prueba = 'giro' if round(node.target_deg) == 360 else f'giro_{node.target_deg:.0f}'
            err_deg = odom_deg - medido
            error_pct = abs(err_deg) / medido * 100.0
            print('\n  ' + '-' * 58)
            print(f'  Variante {node.variante}, prueba {prueba}: real = {medido:.1f} deg, '
                  f'odom = {odom_deg:.1f} deg')
            print(f'  Error: {err_deg:+.1f} deg ({error_pct:.2f} %)')
            print('  ' + '-' * 58)
            append_csv(log_path, CAMPOS_VALIDACION,
                       [node.variante, prueba, f'{medido:.1f}', f'{odom_deg:.1f}',
                        f'{error_pct:.2f}'])
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

        append_csv(log_path, CAMPOS_CALIBRACION,
                   [datetime.now().isoformat(timespec='seconds'), iteration,
                    f'{b_before:.4f}', f'{node.target_deg:.0f}', f'{medido:.1f}',
                    f'{b_after:.4f}', f'{error_pct:.2f}', f'{odom_deg:.1f}'])

        print('\n  SIGUIENTE PASO:')
        print(f'   1) config/wheel_params.yaml -> wheel_separation: {b_after:.4f}')
        print('   2) Relanza el bringup.')
        print(f'   3) Vuelve a correr este script con  -p b_eff:={b_after:.4f}')
        print('   Repite hasta que el error sea < 1 %.')

    except (KeyboardInterrupt, EOFError, SystemExit):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        if validacion:
            print(f'\nRegistro de validacion: {log_path}')
        else:
            print(f'\nLog de calibracion: {log_path}')


if __name__ == '__main__':
    main()
