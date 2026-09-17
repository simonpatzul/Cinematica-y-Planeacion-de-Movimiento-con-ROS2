#!/usr/bin/env bash
# PARTE 4C - PICK -> PRE-PLACE con la pieza adjunta (ATTACHED).
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "PARTE 4C — PICK -> PRE-PLACE (pieza adjunta)"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1
asegurar_moveit || exit 1
recargar_escena > "$R_LOGS/4c_escena.txt" 2>&1

info "Ejecutando HOME -> 4A -> 4B (attach) -> 4C..."
asegurar_sin_pick_place_huerfano
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4C -p results_dir:="$R_ROS2" \
  > "$R_LOGS/4c_terminal.txt" 2>&1
codigo=$?

echo
if grep -q "pieza adjuntada" "$R_LOGS/4c_terminal.txt"; then
  ok "WORKPIECE ATTACHED: SÍ"
else
  mal "WORKPIECE ATTACHED: NO (no se confirmó el attach antes de 4C)"
  codigo=1
fi

echo
grep -E "checklist de intentos|intentos cumplieron|gana |Demostracion|Ciclo detenido" "$R_LOGS/4c_terminal.txt"
echo

csv="$R_ROS2/trayectorias_4C_pick_pre_place.csv"
if [[ -f "$csv" ]]; then
  python3 "$COMMON_DIR/graficar_candidatos.py" "$csv" "$R_IMAGENES/comparacion_planeadores_4C.png" \
    "4C: PICK -> PRE-PLACE (pieza adjunta)"
else
  mal "No se generó $csv"
  codigo=1
fi

echo
echo "Evidencia: $csv"
echo "           $R_LOGS/4c_terminal.txt"
echo "           $R_IMAGENES/comparacion_planeadores_4C.png"

[[ $codigo -eq 0 ]] && ok "PARTE 4C: OK" || mal "PARTE 4C: FALLÓ"
exit $codigo
