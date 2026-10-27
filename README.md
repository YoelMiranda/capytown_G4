# capytown_esan — RC-0 "Primer arranque y calibración"

Paquete ROS 2 Humble (ament_python) para el Yahboom MicroROS-Pi5: odometría corregible
(r, b_eff), calibración, grabación de bags y gráficas. Proyecto CapyTown, Robótica ESAN 2026-2.

## Integrantes y roles

| Rol | Integrante | Responsabilidad en RC-0 |
|---|---|---|
| Integración | `<COMPLETAR>` | Bringup, ROS_DOMAIN_ID, instalación de rosbag2, bags, repositorio y tag |
| Control | `<COMPLETAR>` | Calibración de r y b_eff; actualización de `wheel_params.yaml` |
| Planificación | `<COMPLETAR>` | `scripts/rc0_pruebas.sh`: secuencia de pruebas (recta y giros) |
| Percepción | `<COMPLETAR>` | `plot_odom.py`: lectura de bags y gráficas |

## Sección y variantes (S-003)

| Uso | Variante | Recta de 1.0 m | Giro en sitio |
|---|---|---|---|
| Calibración (Partes B y C) | **A1 "Torre"** | B4 → C4, de oeste a este | tile B2, **antihorario** (`angular_vel = +0.5`) |
| Validación sin recalibrar (Parte D) | **A2 "Escalón"** | C4 → D4, de este a oeste | tile A1, **horario** (`angular_vel = −0.5`) |

Bonus (en A2, frente al docente): giro de 720° con error ≤ 5° y recta de 1.0 m con error ≤ 1 %.

## ROS_DOMAIN_ID

**ROS_DOMAIN_ID = 20**, igual en la Pi5 y en la placa. Se exporta en **cada** terminal:

```bash
export ROS_DOMAIN_ID=20
```

## Parámetros finales

| Parámetro | Valor final | Inicial |
|---|---|---|
| `wheel_radius` (r) | `<COMPLETAR>` m | 0.0325 m |
| `wheel_separation` (b_eff) | `<COMPLETAR>` m | 0.2400 m |

`firmware_wheel_radius` (r0) y `firmware_wheel_separation` (b0) son los valores que asume el
ESP32 y no se tocan. `odom_corrector` reescala `/odom_raw` → `/odom`:
`ds' = ds·r/r0`, `dθ' = dθ·(r/r0)·(b0/b_eff)`. El firmware siempre usa r0 y b0, así que r y
b_eff solo corrigen la odometría; no cambian cómo se mueve el robot.

## Arquitectura

```
ESP32 (firmware) ──serial──> micro_ros_agent (contenedor aparte) ──DDS, dominio 20──┐
   publica /odom_raw (~11 Hz), /imu, /scan, /battery; escucha /cmd_vel              │
                                                                                     v
                                 odom_corrector (bringup.launch.py) ──> /odom + tf odom->base_footprint
```

**El agente micro-ROS no lo lanza `bringup.launch.py`.** Corre como un contenedor aparte
(`micro_ros_agent`) y hay que levantarlo antes. Tampoco se incluye el bringup de Yahboom,
porque su EKF publicaría otro `/odom`.

## Arranque paso a paso (Parte A)

1. **Agente micro-ROS** (en la Pi, fuera del contenedor de ROS):
   ```bash
   sh start_agent_rpi5.sh
   ```
   Luego **resetear la placa** (botón RESET del ESP32 o `sh ~/reset_esp32.sh`) para que se
   conecte al agente. Comprobar con `docker ps` que `micro_ros_agent` está arriba.

2. **Entrar al contenedor de ROS 2**:
   ```bash
   sh ros2_humble.sh
   ```

3. **Instalar rosbag2** (una vez por contenedor, como root; Anexo A del enunciado):
   ```bash
   apt-key del F42ED6FBAB17C654        # si dice "No such key", continuar
   apt-get update; apt-get install -y curl
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list
   apt-get clean && apt-get update
   apt-get install -y ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap
   ros2 bag record --list-storage       # debe listar sqlite3 y mcap
   ```
   Para no repetirlo, guardar el contenedor con `docker commit`.

4. **Compilar** (el repo va dentro de `src/`; colcon toma el nombre del paquete de `package.xml`):
   ```bash
   # en la Pi, fuera del contenedor
   cp -r capytown_G4 ~/yahboomcar_ws/src/
   # dentro del contenedor
   cd /root/yahboomcar_ws && colcon build --packages-select capytown_esan --symlink-install
   source install/setup.bash
   ```

5. **Bringup**:
   ```bash
   export ROS_DOMAIN_ID=20
   ros2 launch capytown_esan bringup.launch.py          # publica /odom a ~11 Hz
   ```

6. **Verificación y `setup_log.txt`** (otra terminal, desde la raíz del repo):
   ```bash
   export ROS_DOMAIN_ID=20
   bash scripts/rc0_pruebas.sh check
   ```
   Anexa a `setup_log.txt` la fecha, `ros2 node list`, `ros2 topic list` y la frecuencia de
   `/odom`, `/imu` y `/odom_raw`. **La captura de `rqt_graph` se agrega a mano.**

Si no llega `/odom_raw`, revisar `docker ps` (micro_ros_agent), `ROS_DOMAIN_ID=20` y resetear el
ESP32 (`sh ~/reset_esp32.sh` en la Pi).

## Procedimiento de pruebas (Partes B a E)

Todo se corre **desde la raíz del repo**, con el bringup corriendo y `ROS_DOMAIN_ID=20`.
`scripts/rc0_pruebas.sh --help` muestra la ayuda. Los bags se guardan en `bags/` y nunca se
sobrescriben: si el nombre existe, se usa `_2`, `_3`, …

### Parte B: recta de 1.0 m en A1, calibración de r

```bash
bash scripts/rc0_pruebas.sh recta A1 rc0_recta     # 3 corridas: rc0_recta, rc0_recta_2, rc0_recta_3
```

El script muestra la recta y el sentido de la variante, espera ENTER, resetea la pose y graba
`/odom /odom_raw /cmd_vel /tf`. Luego comanda 0.10 m/s × 100 mensajes a 10 Hz (10 s), frena,
cierra el bag con SIGINT y muestra `d_odom` y el desvío lateral con `plot_odom`. Con cinta se
mide `d_real` y el desvío, y se anota una fila en `linea_recta.csv`
(`corrida, r, d_real, d_odom, error_%, desvio_lateral`). Si el error es > 2 %, corregir
`wheel_radius` en `config/wheel_params.yaml`, relanzar el bringup y repetir.

### Parte C: giro de 360° en A1, calibración de b_eff

```bash
bash scripts/rc0_pruebas.sh giro A1 calibracion 360 rc0_giro_it1   # iteraciones intermedias
bash scripts/rc0_pruebas.sh giro A1                                # último giro -> bags/rc0_giro
```

El script lee el b_eff activo con `ros2 param get /odom_corrector wheel_separation`, graba el
bag y corre `calibrate_beff` con `angular_vel = +0.5` (antihorario). El ángulo medido con
transportador se ingresa **siempre positivo**. Cada iteración queda en `calibration_log.csv`
(con la columna `odom_deg`). Iterar hasta un error < 1 %, escribir el b_eff final en
`wheel_params.yaml` y relanzar el bringup.

`calibrate_beff` comprueba al iniciar que `-p b_eff` coincida con `wheel_separation` de
`/odom_corrector` y aborta si no. Si de verdad se quiere continuar: `-p forzar:=true`.

### Parte D: validación en A2, sin recalibrar

```bash
bash scripts/rc0_pruebas.sh recta A2 rc0_val_recta   # 2 corridas: rc0_val_recta, rc0_val_recta_2
bash scripts/rc0_pruebas.sh giro A2                  # 360° horario -> bags/rc0_val_giro
```

El giro en modo validación **no calcula un b_eff nuevo**: escribe directamente en
`validacion.csv` (`variante, prueba, valor_real, valor_odom, error_%`), con
`error_% = |odom − real| / real · 100`. Las rectas se anotan a mano en `validacion.csv`
(prueba `recta`, `valor_real` = cinta, `valor_odom` = `d_odom`).

**Bonus** (720° horario, error ≤ 5°):

```bash
bash scripts/rc0_pruebas.sh giro A2 bonus            # -> bags/rc0_val_giro_720, prueba giro_720
```

### Parte E: gráficas

```bash
ros2 run capytown_esan plot_odom bags/rc0_recta bags/rc0_giro bags/rc0_val_recta bags/rc0_val_giro --out plots
```

Por cada bag genera `plots/<bag>_xy.png` (trayectoria con inicio, fin y heading inicial) y
`plots/<bag>_theta.png` (θ(t) desenrollado). Imprime una tabla con duración, n_msgs, d_odom,
desvío lateral y giro acumulado. El formato (sqlite3 o mcap) se lee del `metadata.yaml` de
cada bag. Opciones:

- `--topic /odom_raw`: graficar la odometría sin corregir.
- `--tipo recta|giro`: forzar el tipo de prueba (por defecto, `auto`).
- `--resumen plots/resumen.csv`: anexar las métricas a un CSV aparte. No se puede usar
  `linea_recta.csv` porque tiene otras columnas.

## Reglas de calibración

- Recta de 1.0 m: `r_nuevo = r · d_real / d_odom` → `wheel_radius`.
- Giro de 360°: `b_eff_nuevo = b_eff · θ_objetivo / θ_medido` → `wheel_separation`.
- Primero r y después b_eff, un parámetro a la vez. Tras editar el YAML, relanzar el bringup
  (o probar sin relanzar con `ros2 param set /odom_corrector wheel_separation 0.26`).
- `firmware_wheel_radius` y `firmware_wheel_separation` no se tocan.

## Comandos sueltos

```bash
ros2 topic pub --once /odom_reset std_msgs/msg/Empty "{}"   # pose de /odom en cero
ros2 run capytown_esan teleop_key                            # posicionar el robot
ros2 run capytown_esan calibrate_beff --ros-args -p b_eff:=0.24 -p target_angle_deg:=360.0 -p angular_vel:=0.5
ros2 bag record /odom /odom_raw /cmd_vel /tf -o bags/rc0_recta
ros2 run capytown_esan error_odom bags/<bag> [lado_m]        # cuadrado (RC-1)
```

Tópicos del robot: `/cmd_vel` (Twist), `/odom_raw` (firmware, ~11 Hz), `/odom` (corregida),
`/imu`, `/scan` y `/battery`.

## Simulación sin robot

```bash
ros2 launch capytown_esan sim.launch.py b_eff_true:=0.27
ros2 run capytown_esan calibrate_beff --ros-args -p b_eff:=0.24
ros2 topic echo /true_yaw_deg      # hace de transportador
```

En simulación, `fake_robot` publica `/odom` directamente: no hay `/odom_raw`, ni
`odom_corrector`, ni `/odom_reset`. `calibrate_beff` y `rc0_pruebas.sh` avisan y continúan; el
b_eff se toma de `config/wheel_params.yaml`.

## Estructura del repositorio y entregable

```
capytown_G4/                       (tag v1.0-rc0)
├── capytown_esan/
│   ├── odom_corrector.py   # /odom_raw -> /odom con r y b_eff del YAML (+ tf odom->base_footprint)
│   ├── calibrate_beff.py   # giro y calculo de b_eff (modo calibracion | validacion)
│   ├── bag_utils.py        # lectura de bags (sqlite3/mcap) y metricas de recta y giro
│   ├── plot_odom.py        # graficas x-y y theta(t) de los bags
│   ├── teleop_key.py       # WASD -> /cmd_vel
│   ├── run_square.py       # cuadrado en lazo abierto (RC-1)
│   ├── error_odom.py       # trayectoria de un bag + error de cierre (RC-1)
│   └── fake_robot.py       # simulador sin hardware
├── config/wheel_params.yaml   # r y b_eff finales
├── launch/
│   ├── bringup.launch.py   # odom_corrector (el agente micro-ROS va aparte)
│   └── sim.launch.py       # fake_robot
├── scripts/rc0_pruebas.sh  # secuencia de pruebas RC-0
├── test/test_bag_utils.py  # pytest de las metricas
├── bags/                   # rc0_recta/, rc0_giro/, rc0_val_recta/, rc0_val_giro/
├── plots/                  # PNG de plot_odom
├── linea_recta.csv         # corrida, r, d_real, d_odom, error_%, desvio_lateral
├── calibration_log.csv     # lo genera calibrate_beff
├── validacion.csv          # variante, prueba, valor_real, valor_odom, error_%
├── setup_log.txt           # lo genera rc0_pruebas.sh check (+ captura de rqt_graph)
├── informe.pdf
└── README.md
```

Tests de las funciones puras (sin robot):

```bash
python3 -m pytest test/test_bag_utils.py -v
```
