#!/usr/bin/env python3
"""Grafica la pose de uno o varios ros2 bags (sqlite3 o mcap) y resume la recta o el giro.
Por cada bag genera <out>/<nombre_bag>_xy.png y <out>/<nombre_bag>_theta.png.

Uso: ros2 run capytown_esan plot_odom <bag> [<bag> ...] [--topic /odom] [--out plots]
                                      [--tipo auto|recta|giro] [--resumen archivo.csv]"""
import argparse
import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from capytown_esan.bag_utils import metricas_giro, metricas_recta, read_pose, topic_counts, unwrap

COLOR_TRAY = '#B85042'
COLOR_REF = '#5C6D3A'

CAMPOS_RESUMEN = ['bag', 'topic', 'tipo', 'duracion_s', 'n_msgs', 'd_odom_m',
                  'avance_m', 'desvio_lateral_m', 'giro_deg']


def analizar(t, x, y, th, tipo='auto'):
    """Metricas de un bag. En modo auto: giro si /odom acumula mas de 90 grados y se
    desplaza menos de 20 cm; si no, recta."""
    rec = metricas_recta(x, y, th)
    giro = metricas_giro(th)
    if tipo == 'auto':
        tipo = 'giro' if abs(giro) > 90.0 and rec['d_odom'] < 0.20 else 'recta'
    return {
        'tipo': tipo,
        'duracion_s': t[-1] - t[0],
        'n_msgs': len(t),
        'd_odom_m': rec['d_odom'],
        'avance_m': rec['avance'],
        'desvio_lateral_m': rec['desvio_lateral'],
        'giro_deg': giro,
    }


def graficar(nombre, topic, t, x, y, th, met, out_dir):
    """Guarda los PNG x-y y theta(t). Devuelve las dos rutas."""
    os.makedirs(out_dir, exist_ok=True)

    # --- trayectoria x-y
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(x, y, '-', color=COLOR_TRAY, lw=1.5, label=f'trayectoria {topic}')
    # heading inicial como referencia: largo de la recta, o 20 cm en un giro
    largo = max(met['d_odom_m'], 0.20)
    hx = x[0] + largo * math.cos(th[0])
    hy = y[0] + largo * math.sin(th[0])
    ax.plot([x[0], hx], [y[0], hy], '--', color=COLOR_REF, lw=1.2, label='heading inicial')
    ax.scatter([x[0]], [y[0]], c='green', s=60, zorder=5, label='inicio')
    ax.scatter([x[-1]], [y[-1]], c='red', s=60, zorder=5, label='fin')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=8)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'{nombre}: trayectoria {topic} ({met["tipo"]})')
    if met['tipo'] == 'recta':
        texto = (f'd_odom = {met["d_odom_m"] * 100:.1f} cm\n'
                 f'desvio lateral = {met["desvio_lateral_m"] * 100:+.1f} cm')
    else:
        texto = (f'giro acumulado = {met["giro_deg"]:+.1f} deg\n'
                 f'deriva = {met["d_odom_m"] * 100:.1f} cm')
    ax.annotate(texto, xy=(0.03, 0.97), xycoords='axes fraction', va='top', fontsize=10,
                bbox=dict(boxstyle='round', fc='wheat', alpha=0.8))
    png_xy = os.path.join(out_dir, f'{nombre}_xy.png')
    fig.savefig(png_xy, dpi=150, bbox_inches='tight')
    plt.close(fig)

    # --- orientacion theta(t) desenrollada
    th_deg = [math.degrees(a) for a in unwrap(th)]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(t, th_deg, '-', color=COLOR_TRAY, lw=1.5, label='theta desenrollado')
    ax.axhline(th_deg[0], ls='--', color=COLOR_REF, lw=1.0, label='theta inicial')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    ax.set_xlabel('t [s]')
    ax.set_ylabel('theta [deg]')
    ax.set_title(f'{nombre}: orientacion {topic}  (giro acumulado {met["giro_deg"]:+.1f} deg)')
    png_th = os.path.join(out_dir, f'{nombre}_theta.png')
    fig.savefig(png_th, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return png_xy, png_th


def imprimir_tabla(filas):
    enc = f'{"bag":<24} {"tipo":<6} {"dur[s]":>7} {"n_msgs":>7} {"d_odom[m]":>10} ' \
          f'{"desvio[cm]":>11} {"giro[deg]":>10}'
    print('\n' + enc)
    print('-' * len(enc))
    for f in filas:
        print(f'{f["bag"]:<24} {f["tipo"]:<6} {f["duracion_s"]:>7.2f} {f["n_msgs"]:>7d} '
              f'{f["d_odom_m"]:>10.4f} {f["desvio_lateral_m"] * 100:>+11.2f} '
              f'{f["giro_deg"]:>+10.1f}')
    print()


def anexar_resumen(path, filas):
    carpeta = os.path.dirname(os.path.abspath(path))
    os.makedirs(carpeta, exist_ok=True)
    es_nuevo = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        w = csv.writer(f)
        if es_nuevo:
            w.writerow(CAMPOS_RESUMEN)
        for r in filas:
            w.writerow([r['bag'], r['topic'], r['tipo'], f'{r["duracion_s"]:.2f}', r['n_msgs'],
                        f'{r["d_odom_m"]:.4f}', f'{r["avance_m"]:.4f}',
                        f'{r["desvio_lateral_m"]:.4f}', f'{r["giro_deg"]:.2f}'])
    print(f'Resumen anexado en: {path}')


def aviso_sin_mensajes(bag, topic):
    print(f'[AVISO] {bag}: el topico {topic} no tiene mensajes.')
    try:
        conteo = topic_counts(bag)
        if conteo:
            print('        Topicos en el bag: ' +
                  ', '.join(f'{k} ({v})' for k, v in sorted(conteo.items())))
    except (OSError, KeyError, TypeError):
        pass
    if topic != '/odom_raw':
        print('        Prueba con  --topic /odom_raw')


def parse_args(argv):
    # ros2 run puede agregar --ros-args ...; no son para este script
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    p = argparse.ArgumentParser(
        prog='ros2 run capytown_esan plot_odom',
        description='Grafica x-y y theta(t) de ros2 bags (sqlite3 o mcap) y resume recta/giro.')
    p.add_argument('bags', nargs='+', help='carpeta(s) de bag')
    p.add_argument('--topic', default='/odom', help='topico nav_msgs/Odometry (def. /odom)')
    p.add_argument('--out', default='plots', help='carpeta de salida de los PNG (def. plots)')
    p.add_argument('--tipo', choices=['auto', 'recta', 'giro'], default='auto',
                   help='tipo de prueba para titulos y anotaciones (def. auto)')
    p.add_argument('--resumen', metavar='archivo.csv',
                   help='anexa una fila por bag con las metricas a este CSV')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    filas = []
    fallos = 0
    for bag in args.bags:
        nombre = os.path.basename(os.path.normpath(bag))
        try:
            t, x, y, th = read_pose(bag, args.topic)
        except FileNotFoundError as e:
            print(f'[ERROR] {e}')
            fallos += 1
            continue
        except Exception as e:  # bag corrupto, plugin mcap no instalado, etc.
            print(f'[ERROR] No se pudo leer {bag}: {e}')
            print('        Si es mcap: apt-get install -y ros-humble-rosbag2-storage-mcap')
            fallos += 1
            continue
        if len(t) < 2:
            aviso_sin_mensajes(bag, args.topic)
            fallos += 1
            continue

        met = analizar(t, x, y, th, args.tipo)
        png_xy, png_th = graficar(nombre, args.topic, t, x, y, th, met, args.out)
        print(f'{nombre}: {png_xy}  {png_th}')
        met.update({'bag': nombre, 'topic': args.topic})
        filas.append(met)

    if filas:
        imprimir_tabla(filas)
        if args.resumen:
            anexar_resumen(args.resumen, filas)
    return 1 if fallos else 0


if __name__ == '__main__':
    sys.exit(main())
