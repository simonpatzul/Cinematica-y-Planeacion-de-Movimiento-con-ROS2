#!/usr/bin/env bash
# Variables y funciones compartidas por todos los scripts taller_*.sh.
# Se asume "set -u" en el caller; no se activa "set -e" aqui para que cada
# comando pueda decidir cuando un fallo real debe detener la ejecucion.

# ROOT_DIR se calcula desde la ubicacion de este archivo, no desde el
# directorio donde se invoco taller.sh, para que el repositorio funcione
# clonado en cualquier ruta.
COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$COMMON_DIR/.." && pwd)"

RESULTADOS="$ROOT_DIR/resultados"
R_ROS2="$RESULTADOS/ros2"
R_LOGS="$RESULTADOS/logs"
R_TABLAS="$RESULTADOS/tablas"
R_IMAGENES="$RESULTADOS/imagenes"
R_MATLAB="$RESULTADOS/matlab"
R_HISTORICO="$RESULTADOS/historico"

mkdir -p "$R_ROS2" "$R_LOGS" "$R_TABLAS" "$R_IMAGENES" "$R_MATLAB" "$R_HISTORICO"

C_VERDE='\033[0;32m'
C_ROJO='\033[0;31m'
C_AMARILLO='\033[1;33m'
C_AZUL='\033[0;34m'
C_RESET='\033[0m'

linea() { printf '%s\n' "============================================"; }

titulo() {
  linea
  printf '%s\n' "$1"
  linea
}

ok()   { printf "${C_VERDE}[OK]${C_RESET} %s\n" "$1"; }
mal()  { printf "${C_ROJO}[FALLO]${C_RESET} %s\n" "$1"; }
info() { printf "${C_AZUL}[INFO]${C_RESET} %s\n" "$1"; }
aviso(){ printf "${C_AMARILLO}[AVISO]${C_RESET} %s\n" "$1"; }

# Carga el entorno de ROS 2 Jazzy + el overlay del workspace, si existen.
cargar_ros() {
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    mal "No existe /opt/ros/jazzy/setup.bash. Instale ROS 2 Jazzy o actívelo antes de continuar."
    return 1
  fi
  # Los scripts de ROS2 usan variables sin inicializar; "set -u" del caller
  # los rompe, asi que se desactiva solo mientras se cargan.
  local u_estaba_activo=0
  case "$-" in *u*) u_estaba_activo=1 ;; esac
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROOT_DIR/install/setup.bash"
  else
    [[ $u_estaba_activo -eq 1 ]] && set -u
    mal "No existe $ROOT_DIR/install/setup.bash. Ejecute primero: ./taller.sh compilar"
    return 1
  fi
  [[ $u_estaba_activo -eq 1 ]] && set -u
  return 0
}

# Verifica que move_group este activo; si no, lanza demo.launch.py en
# segundo plano y espera a que quede listo. Si ya estaba activo, no relanza
# nada (evita procesos duplicados entre comandos consecutivos).
asegurar_moveit() {
  if ros2 node list 2>/dev/null | grep -q "/move_group"; then
    info "MoveIt2 ya está activo, se reutiliza la sesión existente."
    return 0
  fi

  info "MoveIt2 no está activo. Lanzando demo.launch.py en segundo plano..."
  mkdir -p "$R_LOGS"
  local log="$R_LOGS/demo_launch.log"
  (cd "$ROOT_DIR" && QT_QPA_PLATFORM=xcb ros2 launch ur5_moveit_config demo.launch.py > "$log" 2>&1 & disown)

  local intentos=0
  until grep -q "You can start planning now" "$log" 2>/dev/null; do
    sleep 2
    intentos=$((intentos + 1))
    if [[ "$intentos" -ge 30 ]]; then
      mal "MoveIt2 no arrancó a tiempo (60 s). Revise $log"
      return 1
    fi
  done
  info "move_group listo. Esperando controladores..."

  intentos=0
  until ros2 control list_controllers 2>/dev/null | grep -q "active"; do
    sleep 2
    intentos=$((intentos + 1))
    if [[ "$intentos" -ge 20 ]]; then
      mal "Los controladores no activaron a tiempo. Revise $log"
      return 1
    fi
  done
  ok "MoveIt2 + controladores activos."
  return 0
}

# Si una corrida anterior de pick_place_sequence quedó huérfana (p.ej. por un
# "timeout" externo que mató a taller.sh pero no a sus nietos), un segundo
# proceso corriendo a la vez confunde al current_state_monitor y a la
# PlanningScene compartidos con move_group. Se mata cualquier instancia previa
# antes de lanzar una nueva.
asegurar_sin_pick_place_huerfano() {
  if pgrep -f "pick_place_sequence --ros-args" > /dev/null 2>&1; then
    aviso "Había una instancia previa de pick_place_sequence corriendo; se termina antes de continuar."
    pkill -9 -f "pick_place_sequence --ros-args" 2>/dev/null
    sleep 1
  fi
}

# Ejecuta planning_scene_setup (clear + aplicar) para dejar la escena en un
# estado conocido antes de una prueba.
recargar_escena() {
  ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true > /dev/null 2>&1
  ros2 run ur5_pick_place planning_scene_setup
}

# Extrae un valor numerico de un TXT de kinematics_report dado el prefijo de
# la linea (ej: "Error de posicion [m]: 0.000000012").
extraer_valor() {
  local archivo="$1" prefijo="$2"
  grep "$prefijo" "$archivo" | head -1 | sed -E "s/^${prefijo}//" | tr -d ' '
}

# Compara un valor absoluto contra un umbral usando awk (sin depender de bc).
cumple_umbral() {
  local valor="$1" umbral="$2"
  awk -v v="$valor" -v u="$umbral" 'BEGIN { v = (v < 0 ? -v : v); exit !(v <= u) }'
}
