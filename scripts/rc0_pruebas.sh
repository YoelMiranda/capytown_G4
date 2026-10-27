#!/usr/bin/env bash
# Secuencia de pruebas RC-0 (rol Planificacion). Seccion S-003:
#   A1 "Torre"   (calibracion): recta B4-C4 de oeste a este, giro en B2 antihorario (+0.5)
#   A2 "Escalon" (validacion):  recta C4-D4 de este a oeste, giro en A1 horario    (-0.5)
# Asume que el bringup ya corre y que ROS_DOMAIN_ID=20 (solo se verifica, no se cambia).
set -euo pipefail

USO="Uso: rc0_pruebas.sh <subcomando> [args]

Subcomandos:
  recta <A1|A2> <nombre_bag>
      Resetea la pose, graba bags/<nombre_bag>, avanza 0.10 m/s durante 10 s
      (100 msgs a 10 Hz), frena y muestra el resumen con plot_odom.
      Ej.: rc0_pruebas.sh recta A1 rc0_recta     rc0_pruebas.sh recta A2 rc0_val_recta

  giro <A1|A2> [modo] [angulo] [nombre_bag]
      modo: calibracion | validacion | bonus
            (por defecto A1 -> calibracion, A2 -> validacion)
      angulo: 360 por defecto; 720 en modo bonus.
      Sentido segun la variante: A1 -> +0.5 rad/s (antihorario), A2 -> -0.5 (horario).
      Lee b_eff de /odom_corrector, graba el bag (rc0_giro, rc0_val_giro o
      rc0_val_giro_720) y corre calibrate_beff con los parametros correctos.
      Ej.: rc0_pruebas.sh giro A1      rc0_pruebas.sh giro A2      rc0_pruebas.sh giro A2 bonus

  check
      Anexa a setup_log.txt: fecha, ros2 node list, ros2 topic list y la
      frecuencia de /odom, /imu y /odom_raw (10 s cada una).

Si bags/<nombre_bag> ya existe no se sobrescribe: se usa <nombre_bag>_2, _3, ...
Los bags, plots/ y los CSV se escriben en la raiz del repo (donde esta config/)."

# ------------------------------------------------------------------ utilidades

# raiz del repo: carpeta padre de scripts/ (resuelve el symlink de --symlink-install);
# si el script corre desde install/ sin el codigo fuente al lado, se usa el directorio actual
SCRIPT_REAL="$(readlink -f "${BASH_SOURCE[0]}")"
BASE="$(cd "$(dirname "$SCRIPT_REAL")/.." && pwd)"
if [ ! -f "$BASE/config/wheel_params.yaml" ]; then
    BASE="$(pwd)"
fi

REC_PID=""

die() { echo "[ERROR] $*" >&2; exit 1; }

verificar_entorno() {
    command -v ros2 >/dev/null 2>&1 || die "no se encuentra ros2. Ejecuta: source install/setup.bash"
    if [ "${ROS_DOMAIN_ID:-}" != "20" ]; then
        echo "[AVISO] ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-<sin definir>}' y el grupo usa 20."
        echo "        En esta terminal: export ROS_DOMAIN_ID=20"
    fi
}

info_variante() {
    case "$1" in
        A1) echo "Variante A1 \"Torre\" (calibracion): recta B4-C4 de OESTE a ESTE (1.0 m); giro en el tile B2, ANTIHORARIO." ;;
        A2) echo "Variante A2 \"Escalon\" (validacion): recta C4-D4 de ESTE a OESTE (1.0 m); giro en el tile A1, HORARIO." ;;
        *) die "variante '$1' invalida: usa A1 o A2" ;;
    esac
}

dir_unico() {  # bags/<nombre>, o bags/<nombre>_2, _3, ... si ya existe
    local base="$BASE/bags/$1" d n=2
    d="$base"
    while [ -e "$d" ]; do
        d="${base}_${n}"
        n=$((n + 1))
    done
    echo "$d"
}

frenar() {
    local i
    for i in 1 2 3; do
        timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
    done
}

resetear_pose() {
    echo ">> Pose de /odom en cero"
    # --once espera a un suscriptor; en sim (fake_robot) nadie escucha /odom_reset
    timeout 5 ros2 topic pub --once /odom_reset std_msgs/msg/Empty "{}" >/dev/null 2>&1 || \
        echo "[AVISO] nadie atendio /odom_reset (odom_corrector no corre?); la pose no se reseteo"
}

iniciar_bag() {  # $1 = carpeta de salida
    echo ">> Grabando bag en $1"
    mkdir -p "$BASE/bags"
    # job control solo para este proceso: en un script los procesos en segundo plano
    # ignoran SIGINT; con set -m tienen su propio grupo y se pueden cerrar con SIGINT
    set -m
    ros2 bag record /odom /odom_raw /cmd_vel /tf -o "$1" &
    REC_PID=$!
    set +m
}

detener_bag() {
    [ -n "$REC_PID" ] || return 0
    echo ">> Cerrando el bag (SIGINT)..."
    kill -INT -- "-$REC_PID" 2>/dev/null || kill -INT "$REC_PID" 2>/dev/null || true
    local i
    for i in $(seq 1 30); do
        kill -0 "$REC_PID" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 "$REC_PID" 2>/dev/null; then
        echo "[AVISO] el record no cerro en 15 s; se envia SIGTERM"
        kill -TERM -- "-$REC_PID" 2>/dev/null || kill -TERM "$REC_PID" 2>/dev/null || true
    fi
    wait "$REC_PID" 2>/dev/null || true
    REC_PID=""
}

limpiar() {  # Ctrl+C o error: robot quieto y bag cerrado
    if [ -n "$REC_PID" ]; then
        echo
        echo ">> Interrumpido: frenando el robot y cerrando el bag"
        frenar
        detener_bag
    fi
}
trap limpiar EXIT
trap 'exit 130' INT TERM

esperar_enter() {
    read -r -p "Coloca el robot y pulsa ENTER para iniciar (Ctrl+C para cancelar)... " _
}

resumen_plot() {  # $1 = carpeta del bag, $2 = tipo
    echo ">> Resumen del bag"
    ros2 run capytown_esan plot_odom "$1" --tipo "$2" --out "$BASE/plots" || \
        echo "[AVISO] plot_odom no pudo leer el bag; revisa ros2 bag info $1"
}

# ------------------------------------------------------------------ subcomandos

cmd_recta() {
    [ $# -eq 2 ] || die "uso: rc0_pruebas.sh recta <A1|A2> <nombre_bag>"
    local variante="$1" dir
    info_variante "$variante" >/dev/null
    verificar_entorno
    dir="$(dir_unico "$2")"

    echo "================================================================"
    echo " RECTA DE 1.0 m"
    info_variante "$variante"
    echo " Eje de las ruedas sobre la linea de inicio, centrado en el carril."
    echo " Bag: $dir"
    echo "================================================================"
    esperar_enter

    resetear_pose
    iniciar_bag "$dir"
    sleep 2
    echo ">> Avanzando 0.10 m/s durante 10 s (100 msgs a 10 Hz)"
    ros2 topic pub --rate 10 --times 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}}" >/dev/null
    frenar
    sleep 1
    detener_bag

    resumen_plot "$dir" recta
    echo "Mide con cinta d_real y el desvio lateral, y anotalos en linea_recta.csv (o validacion.csv en A2)."
}

leer_beff() {
    local salida b
    if salida="$(timeout 10 ros2 param get /odom_corrector wheel_separation 2>/dev/null)"; then
        b="$(echo "$salida" | awk '{print $NF}')"
        if [[ "$b" =~ ^[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$ ]]; then
            echo "$b"
            return 0
        fi
    fi
    echo "[AVISO] /odom_corrector no responde; se usa wheel_separation de config/wheel_params.yaml" >&2
    python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']['wheel_separation'])" \
        "$BASE/config/wheel_params.yaml"
}

cmd_giro() {
    [ $# -ge 1 ] && [ $# -le 4 ] || die "uso: rc0_pruebas.sh giro <A1|A2> [modo] [angulo] [nombre_bag]"
    local variante="$1" modo vel angulo nombre modo_calib log_file b_eff dir
    info_variante "$variante" >/dev/null

    case "$variante" in
        A1) vel="0.5";  modo="${2:-calibracion}" ;;
        A2) vel="-0.5"; modo="${2:-validacion}" ;;
    esac
    case "$modo" in
        calibracion) angulo="${3:-360}"; nombre="${4:-rc0_giro}";     modo_calib="calibracion"; log_file="calibration_log.csv" ;;
        validacion)  angulo="${3:-360}"; nombre="${4:-rc0_val_giro}"; modo_calib="validacion";  log_file="validacion.csv" ;;
        bonus)       angulo="${3:-720}"; nombre="${4:-rc0_val_giro_${angulo}}"; modo_calib="validacion"; log_file="validacion.csv" ;;
        *) die "modo '$modo' invalido: usa calibracion, validacion o bonus" ;;
    esac
    [[ "$angulo" =~ ^[0-9]+(\.[0-9]+)?$ ]] || die "angulo '$angulo' invalido (grados, positivo)"
    # target_angle_deg es DOUBLE en calibrate_beff: un entero (720) haria fallar el nodo
    [[ "$angulo" == *.* ]] || angulo="${angulo}.0"
    if [ "$variante" = "A1" ] && [ "$modo" != "calibracion" ]; then
        echo "[AVISO] En S-003 la validacion y el bonus se hacen en A2."
    fi
    if [ "$variante" = "A2" ] && [ "$modo" = "calibracion" ]; then
        echo "[AVISO] En S-003 se calibra en A1; en A2 no se recalibra."
    fi

    verificar_entorno
    b_eff="$(leer_beff)"
    dir="$(dir_unico "$nombre")"

    echo "================================================================"
    echo " GIRO DE ${angulo} grados (modo ${modo})"
    info_variante "$variante"
    echo " angular_vel = ${vel} rad/s   b_eff activo = ${b_eff} m"
    echo " Registro: ${log_file}   Bag: ${dir}"
    echo " Marca con cinta la posicion y la orientacion inicial del robot."
    echo "================================================================"

    resetear_pose
    iniciar_bag "$dir"
    sleep 2
    # calibrate_beff escribe el CSV en el directorio actual
    (cd "$BASE" && ros2 run capytown_esan calibrate_beff --ros-args \
        -p b_eff:="$b_eff" -p target_angle_deg:="$angulo" -p angular_vel:="$vel" \
        -p modo:="$modo_calib" -p variante:="$variante" -p log_file:="$log_file") || true
    frenar
    detener_bag

    resumen_plot "$dir" giro
}

cmd_check() {
    verificar_entorno
    local log="$BASE/setup_log.txt"
    {
        echo "================================================================"
        echo " setup_log RC-0 -- $(date '+%Y-%m-%d %H:%M:%S')"
        echo " ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<sin definir>}"
        echo "================================================================"
        echo
        echo "\$ ros2 node list"
        ros2 node list 2>&1 || true
        echo
        echo "\$ ros2 topic list"
        ros2 topic list 2>&1 || true
        local t
        for t in /odom /imu /odom_raw; do
            echo
            echo "\$ timeout 10 ros2 topic hz $t"
            timeout 10 ros2 topic hz "$t" 2>&1 || true
        done
        echo
        echo "RECORDATORIO: agregar a mano la captura de rqt_graph (rqt_graph -> guardar imagen)."
        echo
    } | tee -a "$log"
    echo "Guardado en: $log"
}

# ------------------------------------------------------------------ main

[ $# -ge 1 ] || { echo "$USO"; exit 1; }
sub="$1"
shift
case "$sub" in
    -h|--help|help) echo "$USO" ;;
    recta) cmd_recta "$@" ;;
    giro)  cmd_giro "$@" ;;
    check) cmd_check ;;
    *) echo "$USO"; die "subcomando '$sub' desconocido" ;;
esac
