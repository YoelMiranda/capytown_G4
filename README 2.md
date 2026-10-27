# capytown_esan

Paquete ROS 2 Humble (ament_python) para el Yahboom MicroROS-Pi5: odometría corregible
(r, b_eff), calibración y teleoperación. Reto RC-0 CapyTown.

```
capytown_esan/
├── capytown_esan/
│   ├── odom_corrector.py   # /odom_raw -> /odom con r y b_eff del YAML (+ tf odom->base_footprint)
│   ├── calibrate_beff.py   # giro de 360° y cálculo de b_eff
│   ├── teleop_key.py       # WASD -> /cmd_vel
│   ├── run_square.py       # cuadrado en lazo abierto + gráfica
│   ├── error_odom.py       # trayectoria de un bag + error de cierre
│   └── fake_robot.py       # simulador sin hardware
├── config/wheel_params.yaml   # wheel_radius (r), wheel_separation (b_eff), firmware_*
└── launch/
    ├── bringup.launch.py   # odom_corrector con wheel_params.yaml
    └── sim.launch.py       # fake_robot
```

## Instalación en el carro

```bash
# en la Pi (fuera del contenedor), como pi
cp -r capytown_esan ~/yahboomcar_ws/src/

# dentro del contenedor
sh ~/ros2_humble.sh
cd /root/yahboomcar_ws && colcon build --packages-select capytown_esan --symlink-install
source install/setup.bash
```

## Uso

```bash
export ROS_DOMAIN_ID=20
ros2 launch capytown_esan bringup.launch.py          # publica /odom a ~11 Hz
ros2 topic pub --once /odom_reset std_msgs/msg/Empty "{}"   # pose a cero
ros2 run capytown_esan teleop_key
ros2 run capytown_esan calibrate_beff --ros-args -p b_eff:=0.24 -p target_angle_deg:=360.0 -p angular_vel:=0.5
ros2 param set /odom_corrector wheel_separation 0.26  # probar sin relanzar
ros2 bag record /odom /odom_raw /cmd_vel /tf -o rc0_recta
```

Topics del robot: `/cmd_vel` (Twist), `/odom_raw` (firmware, ~11 Hz), `/odom` (corregida),
`/imu`, `/scan`, `/battery`. ROS_DOMAIN_ID fijo en 20.

## Calibración

- Recta de 1.0 m: `r_nuevo = r * d_real / d_odom` → `wheel_radius`.
- Giro de 360°: `b_eff_nuevo = b_eff * 360 / angulo_medido` → `wheel_separation`.
- Un parámetro a la vez; tras editar el YAML, relanzar el bringup.
- `firmware_wheel_radius` y `firmware_wheel_separation` no se tocan.

## Simulación sin robot

```bash
ros2 launch capytown_esan sim.launch.py b_eff_true:=0.27
ros2 run capytown_esan calibrate_beff --ros-args -p b_eff:=0.24
ros2 topic echo /true_yaw_deg      # hace de transportador
```
