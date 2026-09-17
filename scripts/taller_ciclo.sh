#!/usr/bin/env bash
# Ciclo completo: HOME -> PRE-PICK -> PICK -> ATTACH -> PRE-PLACE -> PLACE -> DETACH.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "CICLO COMPLETO PICK-AND-PLACE"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1
asegurar_moveit || exit 1
recargar_escena > "$R_LOGS/ciclo_escena.txt" 2>&1

log="$R_LOGS/ciclo_terminal_final.txt"
: > "$log"

info "Ejecutando el ciclo FULL (HOME -> PRE-PICK -> PICK -> ATTACH -> PRE-PLACE -> PLACE -> DETACH)..."
asegurar_sin_pick_place_huerfano
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=FULL -p results_dir:="$R_ROS2" \
  >> "$log" 2>&1 &
pid_ciclo=$!

echo "[1/7] HOME"
declare -A visto
marcar() {
  local n="$1" etiqueta="$2"
  if [[ -z "${visto[$n]:-}" ]]; then
    echo "[$n/7] $etiqueta"
    visto[$n]=1
  fi
}

while kill -0 "$pid_ciclo" 2>/dev/null; do
  grep -q "4A_home_pre_pick: checklist" "$log" 2>/dev/null && marcar 2 "PRE-PICK (planeando)"
  grep -q "4A_home_pre_pick: gana" "$log" 2>/dev/null && marcar 2 "PRE-PICK alcanzado"
  grep -q "4B_pre_pick_pick: se ejecuta" "$log" 2>/dev/null && marcar 3 "PICK (descenso cartesiano)"
  grep -q "pieza adjuntada" "$log" 2>/dev/null && { marcar 3 "PICK alcanzado"; marcar 4 "ATTACH"; }
  grep -q "4C_pick_pre_place: checklist" "$log" 2>/dev/null && marcar 5 "PRE-PLACE (planeando)"
  grep -q "4C_pick_pre_place: gana" "$log" 2>/dev/null && marcar 5 "PRE-PLACE alcanzado"
  grep -q "4D_pre_place_place: se ejecuta" "$log" 2>/dev/null && marcar 6 "PLACE (descenso cartesiano)"
  grep -q "pieza liberada despues de alcanzar PLACE" "$log" 2>/dev/null && { marcar 6 "PLACE alcanzado"; marcar 7 "DETACH"; }
  sleep 1
done

wait "$pid_ciclo"
codigo=$?

# Pasada final: el proceso puede terminar justo entre la última revisión del
# bucle y el "wait", dejando marcadores sin detectar por una condición de carrera.
grep -q "4A_home_pre_pick: checklist" "$log" 2>/dev/null && marcar 2 "PRE-PICK (planeando)"
grep -q "4A_home_pre_pick: gana" "$log" 2>/dev/null && marcar 2 "PRE-PICK alcanzado"
grep -q "4B_pre_pick_pick: se ejecuta" "$log" 2>/dev/null && marcar 3 "PICK (descenso cartesiano)"
grep -q "pieza adjuntada" "$log" 2>/dev/null && { marcar 3 "PICK alcanzado"; marcar 4 "ATTACH"; }
grep -q "4C_pick_pre_place: checklist" "$log" 2>/dev/null && marcar 5 "PRE-PLACE (planeando)"
grep -q "4C_pick_pre_place: gana" "$log" 2>/dev/null && marcar 5 "PRE-PLACE alcanzado"
grep -q "4D_pre_place_place: se ejecuta" "$log" 2>/dev/null && marcar 6 "PLACE (descenso cartesiano)"
grep -q "pieza liberada despues de alcanzar PLACE" "$log" 2>/dev/null && { marcar 6 "PLACE alcanzado"; marcar 7 "DETACH"; }

for n in 2 3 4 5 6 7; do
  [[ -z "${visto[$n]:-}" ]] && echo "[$n/7] (no alcanzado)"
done

echo
if [[ $codigo -eq 0 ]] && grep -q "Ciclo pick-and-place completado" "$log"; then
  ok "CICLO COMPLETO: OK"
else
  mal "CICLO COMPLETO: FALLÓ"
  codigo=1
fi

echo
echo "Evidencia: $log"
if [[ -d "$R_LOGS/reproducibilidad" ]] && ls "$R_LOGS/reproducibilidad"/*.txt >/dev/null 2>&1; then
  n_repro=$(ls "$R_LOGS/reproducibilidad"/*.txt | wc -l)
  info "Además hay $n_repro corridas previas guardadas en $R_LOGS/reproducibilidad/ que demuestran reproducibilidad."
fi

exit $codigo
