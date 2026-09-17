#!/usr/bin/env bash
# PARTE 3 - Cinematica inversa de PICK y PLACE: IK con MoveIt, FK con DH y error.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "PARTE 3 — IK PICK / PLACE"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1
asegurar_moveit || exit 1

resultado=0
declare -A Q

for estado in PICK PLACE; do
  info "Resolviendo IK de $estado con MoveIt/KDL..."
  ros2 run ur5_pick_place kinematics_report --ros-args -p state:="$estado" -p results_dir:="$R_ROS2" \
    > "$R_LOGS/ik_${estado,,}_stdout.txt" 2>&1
  archivo="$R_ROS2/kinematics_${estado}.txt"
  if [[ ! -f "$archivo" ]]; then
    mal "No se generó $archivo"
    resultado=1
    continue
  fi

  xyz=$(grep -A1 "Posicion XYZ MoveIt" "$archivo" | tail -1)
  quat=$(grep -A1 "Cuaternion MoveIt" "$archivo" | tail -1)
  qvals=$(sed -n '/Orden de articulaciones/,/^$/p' "$archivo" | grep "^  q" | awk -F'= ' '{print $2}' | awk '{print $1}' | tr '\n' ' ')
  Q[$estado]="$qvals"
  err_pos=$(extraer_valor "$archivo" "Error de posicion \[m\]: ")
  err_ori=$(extraer_valor "$archivo" "Error de orientacion \[rad\]: ")
  err_mat=$(extraer_valor "$archivo" "Error maximo de la matriz homogenea: ")

  echo
  echo "--- $estado ---"
  echo "XYZ [m]        = $xyz"
  echo "Cuaternion     = $quat"
  echo "q [rad]        = $(echo "$qvals" | tr '\n' ' ')"
  echo "Error posición  = $err_pos m"
  echo "Error orientación = $err_ori rad"
  echo "Error matriz 4x4 = $err_mat"

  if ! cumple_umbral "$err_pos" 0.0001 || ! cumple_umbral "$err_mat" 0.0001; then
    resultado=1
  fi
done

echo
echo "Resumen:"
printf '%-6s | %10s | %10s | %10s | %10s | %10s | %10s\n' "POSE" "q1" "q2" "q3" "q4" "q5" "q6"
for estado in PICK PLACE; do
  # shellcheck disable=SC2183
  printf '%-6s | ' "$estado"
  echo "${Q[$estado]}" | awk '{printf "%10.5f | %10.5f | %10.5f | %10.5f | %10.5f | %10.5f\n", $1,$2,$3,$4,$5,$6}'
done

echo
echo "MoveIt obtiene la IK; esas mismas articulaciones se llevan al modelo DH y su FK se compara con MoveIt."
echo
echo "Evidencia: $R_ROS2/kinematics_PICK.{txt,csv} y kinematics_PLACE.{txt,csv}"

[[ $resultado -eq 0 ]] && ok "RESULTADO: CUMPLE" || mal "RESULTADO: NO CUMPLE"
exit $resultado
