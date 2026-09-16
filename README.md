# Taller UR5 — ROS 2 Jazzy + MoveIt 2

Workspace del taller de **Cinemática y Planeación de Movimiento** para un UR5 clásico. Incluye descripción URDF/Xacro, configuración propia de MoveIt 2, PlanningScene, comparación RRTConnect/RRT*, ciclo pick-and-place, perfiles cúbico/quíntico, validación DH e informe de Jacobianos.

## Requisitos

- Ubuntu con ROS 2 Jazzy.
- MoveIt 2, RViz 2, ros2_control, `tf2_tools`, `xacro` y `check_urdf`.
- Python 3 con NumPy y Matplotlib.
- MATLAB sólo para la comprobación DH académica; `matlab/UR5_DH_VALIDACION.m` no necesita Robotics System Toolbox.

## Compilación

```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## Ejecución rápida

**Terminal 1:**

```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
QT_QPA_PLATFORM=xcb ros2 launch ur5_moveit_config demo.launch.py
```

Espere a que `move_group`, `ur5_arm_controller` y `joint_state_broadcaster` estén activos antes de ejecutar movimientos.

**Terminal 2 — ciclo completo:**

```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=FULL \
  2>&1 | tee resultados/ciclo_terminal_final.txt
```

## Documentación y resultados

La explicación completa, comandos por numeral, evidencia, solución de errores y guion de sustentación están en [GUIA_COMPLETA_TALLER.md](GUIA_COMPLETA_TALLER.md).

Los resultados se guardan en `resultados/`. La carpeta `resultados/historico_ejecucion_2026-09-15/` contiene evidencia real anterior a la última corrección y está marcada como histórica para no confundirla con resultados finales.
