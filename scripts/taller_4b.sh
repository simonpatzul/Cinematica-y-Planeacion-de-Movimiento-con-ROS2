#!/usr/bin/env bash
# PARTE 4B - PRE-PICK -> PICK: computeCartesianPath, perfil cubico vs quintico.
# Requiere pasar antes por HOME->4A (mismo flujo real del taller: primero se
# llega a PRE-PICK y desde ahi se baja a PICK).
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "PARTE 4B — PRE-PICK -> PICK (computeCartesianPath, cúbico vs quíntico)"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1
asegurar_moveit || exit 1
recargar_escena > "$R_LOGS/4b_escena.txt" 2>&1

info "Ejecutando HOME -> 4A -> 4B (el tramo 4B parte realmente de PRE-PICK)..."
asegurar_sin_pick_place_huerfano
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4B -p results_dir:="$R_ROS2" \
  > "$R_LOGS/4b_terminal.txt" 2>&1
codigo=$?

echo
frac=$(grep -oE "computeCartesianPath alcanzo [0-9.]+%" "$R_LOGS/4b_terminal.txt" | tail -1)
if [[ -n "$frac" ]]; then
  echo "$frac"
else
  ok "computeCartesianPath alcanzó ≥99.9% (no se detuvo por fracción parcial)."
fi
grep -E "4B_pre_pick_pick/(cubico|quintico)|se ejecuta solo el perfil|pieza adjuntada|Demostracion|Ciclo detenido" \
  "$R_LOGS/4b_terminal.txt"
echo

csv="$R_ROS2/perfiles_4B_pre_pick_pick.csv"
if [[ -f "$csv" ]]; then
  echo "Tabla real de perfiles (4B):"
  printf '%-10s %-6s %10s %10s %10s %12s\n' "Perfil" "Válido" "T[s]" "vmax_TCP" "amax_TCP" "Motivo"
  python3 - "$csv" <<'PYEOF'
import csv, sys
for f in csv.DictReader(open(sys.argv[1], encoding="utf-8")):
    print(f"{f['perfil']:<10} {f['valido']:<6} {float(f['duracion_s']):>10.4f} "
          f"{float(f['max_velocidad_tcp_m_s']):>10.4f} {float(f['max_aceleracion_tcp_m_s2']):>10.4f} "
          f"{f['motivo']}")
PYEOF
else
  mal "No se generó $csv"
  codigo=1
fi

echo
echo "Restricciones del taller: v <= 0.200 m/s, a <= 0.300 m/s². No se redondean violaciones."
echo
echo "Evidencia:"
echo "  $csv"
echo "  $R_ROS2/perfil_seleccionado_4B_pre_pick_pick.txt"
echo "  $R_IMAGENES/comparacion_perfiles_4B_rojo.png (modelo escalar Python, ver 'taller.sh matlab')"

[[ $codigo -eq 0 ]] && ok "PARTE 4B: OK" || mal "PARTE 4B: FALLÓ"
exit $codigo
