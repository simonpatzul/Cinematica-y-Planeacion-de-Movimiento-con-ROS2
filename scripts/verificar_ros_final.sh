#!/usr/bin/env bash
set -euo pipefail

WS="/home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws"
RESULTADOS="$WS/resultados"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: no existe /opt/ros/jazzy/setup.bash. Este script debe ejecutarse en ROS 2 Jazzy." >&2
  exit 2
fi

cd "$WS"
mkdir -p "$RESULTADOS"
source /opt/ros/jazzy/setup.bash

echo "[1/6] Dependencias"
rosdep install --from-paths src --ignore-src -r -y

echo "[2/6] Compilación limpia"
rm -rf build install log
colcon build --symlink-install 2>&1 | tee "$RESULTADOS/compilacion_final.txt"
source install/setup.bash

echo "[3/6] Xacro + check_urdf"
xacro src/ur5_description/urdf/ur5.urdf.xacro > /tmp/ur5_taller.urdf
check_urdf /tmp/ur5_taller.urdf 2>&1 | tee "$RESULTADOS/check_urdf.txt"

echo "[4/6] Paquetes y ejecutables"
ros2 pkg list | grep -E '^ur5_(description|moveit_config|pick_place)$'
ros2 pkg executables ur5_pick_place | tee "$RESULTADOS/ejecutables_ur5_pick_place.txt"

echo "[5/6] Perfiles Python"
python3 src/ur5_pick_place/scripts/trajectory_profiles.py --output-dir "$RESULTADOS" \
  | tee "$RESULTADOS/perfiles_terminal.txt"

echo "[6/6] Verificación estática"
python3 scripts/verificar_workspace.py | tee "$RESULTADOS/verificacion_estatica.txt"

cat <<'TXT'

VERIFICACIONES NO AUTOMATIZADAS POR ESTE SCRIPT:
1. Terminal 1: QT_QPA_PLATFORM=xcb ros2 launch ur5_moveit_config demo.launch.py
2. Esperar move_group + controladores activos.
3. Terminal 2: cargar PlanningScene y ejecutar 4A, 4B, 4C, 4D/FULL según GUIA_COMPLETA_TALLER.md.
4. Terminal 3: TF y kinematics_report HOME/PICK/PLACE/4B/4D.
5. Guardar captura/video real de RViz.

No presente la carpeta historico_ejecucion_2026-09-15 como evidencia de la versión final.
TXT
