#!/usr/bin/env bash
# PARTE 4D - PRE-PLACE -> PLACE: reutiliza el perfil elegido en 4B y libera la pieza.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "PARTE 4D — PRE-PLACE -> PLACE (perfil de 4B, DETACH)"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1
asegurar_moveit || exit 1
recargar_escena > "$R_LOGS/4d_escena.txt" 2>&1

info "Ejecutando el ciclo completo HOME -> 4A -> 4B -> 4C -> 4D..."
asegurar_sin_pick_place_huerfano
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4D -p results_dir:="$R_ROS2" \
  > "$R_LOGS/4d_terminal.txt" 2>&1
codigo=$?

echo
perfil_4b=$(grep -oE "perfil seleccionado=[a-z]+" "$R_LOGS/4d_terminal.txt" | head -1 | cut -d= -f2)
perfil_4d=$(grep -oE "4D_pre_place_place: se ejecuta solo el perfil [a-z]+" "$R_LOGS/4d_terminal.txt" | tail -1 | awk '{print $NF}')
echo "Perfil seleccionado en 4B: $perfil_4b"
echo "Perfil usado en 4D:        $perfil_4d"

if [[ -n "$perfil_4b" && "$perfil_4b" == "$perfil_4d" ]]; then
  ok "4D reutiliza el mismo perfil de 4B."
else
  mal "4D no coincide con el perfil de 4B."
  codigo=1
fi

csv="$R_ROS2/perfiles_4D_pre_place_place.csv"
if [[ -f "$csv" ]]; then
  echo
  echo "Tabla real de perfiles (4D):"
  printf '%-10s %-6s %10s %10s %10s %12s\n' "Perfil" "Válido" "T[s]" "vmax_TCP" "amax_TCP" "Motivo"
  python3 - "$csv" <<'PYEOF'
import csv, sys
for f in csv.DictReader(open(sys.argv[1], encoding="utf-8")):
    print(f"{f['perfil']:<10} {f['valido']:<6} {float(f['duracion_s']):>10.4f} "
          f"{float(f['max_velocidad_tcp_m_s']):>10.4f} {float(f['max_aceleracion_tcp_m_s2']):>10.4f} "
          f"{f['motivo']}")
PYEOF
fi

echo
echo "Restricciones del taller: v <= 0.100 m/s, a <= 0.020 m/s²."

echo
if grep -q "pieza liberada despues de alcanzar PLACE" "$R_LOGS/4d_terminal.txt"; then
  ok "WORKPIECE ATTACHED: NO (liberada en PLACE)"
else
  mal "WORKPIECE ATTACHED: SÍ (no se confirmó el detach)"
  codigo=1
fi

echo
echo "Evidencia:"
echo "  $csv"
echo "  $R_ROS2/perfil_seleccionado_4D_pre_place_place.txt"
echo "  $R_LOGS/4d_terminal.txt"

[[ $codigo -eq 0 ]] && ok "PARTE 4D: OK" || mal "PARTE 4D: FALLÓ"
exit $codigo
