#!/usr/bin/env bash
# PARTE 4A - HOME -> PRE-PICK: RRTConnect vs RRTstar, 10 intentos cada uno.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "PARTE 4A — HOME -> PRE-PICK (RRTConnect vs RRTstar)"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1
asegurar_moveit || exit 1
recargar_escena > "$R_LOGS/4a_escena.txt" 2>&1

info "Generando 20 candidatos (10 RRTConnect + 10 RRTstar) y ejecutando solo el ganador..."
asegurar_sin_pick_place_huerfano
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4A -p results_dir:="$R_ROS2" \
  > "$R_LOGS/4a_terminal.txt" 2>&1
codigo=$?

echo
grep -E "checklist de intentos|intentos cumplieron|gana |Demostracion|Ciclo detenido" "$R_LOGS/4a_terminal.txt"
echo

csv="$R_ROS2/trayectorias_4A_home_pre_pick.csv"
if [[ -f "$csv" ]]; then
  python3 "$COMMON_DIR/graficar_candidatos.py" "$csv" "$R_IMAGENES/comparacion_planeadores_4A.png" \
    "4A: HOME -> PRE-PICK (RRTConnect vs RRTstar)"
else
  mal "No se generó $csv"
  codigo=1
fi

echo
echo "Evidencia: $csv"
echo "           $R_LOGS/4a_terminal.txt"
echo "           $R_IMAGENES/comparacion_planeadores_4A.png"

[[ $codigo -eq 0 ]] && ok "PARTE 4A: OK" || mal "PARTE 4A: FALLÓ"
exit $codigo
