#!/usr/bin/env bash
# PARTE 2 - Transformacion homogenea en HOME: T por ROS/MoveIt vs T por DH.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "PARTE 2 — TRANSFORMACIÓN HOME"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1
asegurar_moveit || exit 1

info "Llevando el robot a HOME..."
ros2 run ur5_pick_place move_named_state --ros-args -p target:=HOME -p execute:=true > "$R_LOGS/home_move.txt" 2>&1
if [[ $? -ne 0 ]]; then
  mal "No se pudo alcanzar HOME. Ver $R_LOGS/home_move.txt"
  exit 1
fi
ok "HOME alcanzado."

info "Calculando T por MoveIt/TF y T por DH (kinematics_report)..."
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=HOME -p results_dir:="$R_ROS2" > "$R_LOGS/home_kinematics_stdout.txt" 2>&1
archivo="$R_ROS2/kinematics_HOME.txt"
if [[ ! -f "$archivo" ]]; then
  mal "No se generó $archivo. Ver $R_LOGS/home_kinematics_stdout.txt"
  exit 1
fi

q=$(sed -n '/Orden de articulaciones/,/^$/p' "$archivo" | grep "^  q" )
t_ros=$(sed -n '/FK MoveIt base_link -> tool0/,/^$/p' "$archivo" | tail -n +2)
t_dh=$(sed -n '/FK DH modificado base_link -> tool0/,/^$/p' "$archivo" | tail -n +2)
err_pos=$(extraer_valor "$archivo" "Error de posicion \[m\]: ")
err_ori=$(extraer_valor "$archivo" "Error de orientacion \[rad\]: ")
err_mat=$(extraer_valor "$archivo" "Error maximo de la matriz homogenea: ")

echo
echo "q_HOME [rad] ="
echo "$q"
echo
echo "T_ROS (MoveIt/TF) ="
echo "$t_ros"
echo
echo "T_DH ="
echo "$t_dh"
echo
echo "Error posición   = $err_pos m"
echo "Error orientación = $err_ori rad"
echo "Error matriz 4x4  = $err_mat"
echo

if cumple_umbral "$err_pos" 0.0001 && cumple_umbral "$err_mat" 0.0001; then
  ok "RESULTADO: CUMPLE"
  resultado=0
else
  mal "RESULTADO: NO CUMPLE"
  resultado=1
fi

echo
echo "Evidencia: $R_ROS2/kinematics_HOME.txt y kinematics_HOME.csv"

if command -v octave >/dev/null 2>&1 || command -v matlab >/dev/null 2>&1; then
  info "MATLAB/Octave disponible: ejecutando validación DH independiente (matlab/UR5_DH_VALIDACION.m)..."
  if command -v octave >/dev/null 2>&1; then
    octave --no-gui --eval "cd('$ROOT_DIR/matlab'); UR5_DH_VALIDACION" > "$R_MATLAB/validacion_home_octave.txt" 2>&1
  else
    matlab -batch "cd('$ROOT_DIR/matlab'); UR5_DH_VALIDACION" > "$R_MATLAB/validacion_home_octave.txt" 2>&1
  fi
  sed -n '/^HOME$/,/^$/p' "$R_MATLAB/validacion_home_octave.txt"
  echo "Evidencia MATLAB/Octave: $R_MATLAB/validacion_home_octave.txt"
else
  aviso "MATLAB/Octave no disponible en este equipo; se usa solo el resultado de kinematics_report (DH implementado también en C++)."
fi

exit $resultado
