#!/usr/bin/env bash
# PARTE 1 - Modelo del manipulador: URDF/Xacro, check_urdf y configuracion MoveIt2.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "PARTE 1 — MODELO DEL MANIPULADOR"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1

info "Expandiendo Xacro y validando con check_urdf..."
xacro src/ur5_description/urdf/ur5.urdf.xacro > /tmp/ur5_taller.urdf 2>/tmp/ur5_taller_xacro_err.txt
if [[ $? -ne 0 ]]; then
  mal "xacro falló. Ver /tmp/ur5_taller_xacro_err.txt"
  exit 1
fi
check_urdf /tmp/ur5_taller.urdf > "$R_ROS2/check_urdf.txt" 2>&1
codigo_urdf=$?

echo
cat "$R_ROS2/check_urdf.txt"
echo

if [[ $codigo_urdf -eq 0 ]]; then
  ok "check_urdf aceptó el URDF/Xacro."
else
  mal "check_urdf reportó errores."
fi

echo
echo "Cómo se incorporó el UR5 a MoveIt2 (archivos reales del repositorio):"
echo
echo " 1. URDF/Xacro base:      src/ur5_description/urdf/ur5.urdf.xacro"
echo " 2. Setup Assistant:      src/ur5_moveit_config/.setup_assistant"
grep -A2 "urdf:" "$ROOT_DIR/src/ur5_moveit_config/.setup_assistant" | sed 's/^/                          /'
echo " 3. SRDF generado:        src/ur5_moveit_config/config/ur5.srdf"
echo "    Grupo de planeación:"
grep -A1 'group name="ur_manipulator"' "$ROOT_DIR/src/ur5_moveit_config/config/ur5.srdf" | sed 's/^/      /'
echo "    Estados nombrados:"
grep 'group_state name=' "$ROOT_DIR/src/ur5_moveit_config/config/ur5.srdf" | sed 's/^/      /'
n_colisiones=$(grep -c "disable_collisions" "$ROOT_DIR/src/ur5_moveit_config/config/ur5.srdf")
echo "    Matriz de autocolisiones: $n_colisiones pares deshabilitados."
echo " 4. Límites articulares:  src/ur5_moveit_config/config/joint_limits.yaml"
echo " 5. Solver de IK (KDL):   src/ur5_moveit_config/config/kinematics.yaml"
grep "kinematics_solver" "$ROOT_DIR/src/ur5_moveit_config/config/kinematics.yaml" | sed 's/^/      /'
echo " 6. Planeador OMPL:       src/ur5_moveit_config/config/ompl_planning.yaml"
grep "RRTConnectkConfigDefault\|RRTstarkConfigDefault" "$ROOT_DIR/src/ur5_moveit_config/config/ompl_planning.yaml" | sed 's/^/      /'
echo " 7. Controladores MoveIt: src/ur5_moveit_config/config/moveit_controllers.yaml"
echo "    Controladores ros2_control: src/ur5_moveit_config/config/ros2_controllers.yaml"
echo " 8. RViz:                 src/ur5_moveit_config/config/moveit.rviz (ros2 launch ur5_moveit_config demo.launch.py)"
echo
echo "Evidencia guardada en: $R_ROS2/check_urdf.txt"

[[ $codigo_urdf -eq 0 ]] && exit 0 || exit 1
