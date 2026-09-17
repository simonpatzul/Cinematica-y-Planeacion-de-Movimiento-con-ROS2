# Taller UR5 — ROS 2 Jazzy + MoveIt 2

Entrega independiente del taller de cinemática y planeación de movimiento con el UR5. No tiene relación con la entrega KUKA KR-6 de la carpeta [`../KUKA_KR6_MATLAB/`](../KUKA_KR6_MATLAB/README.md); se conserva aparte a propósito.

Configuración propia de MoveIt 2 para un UR5 clásico construida desde su URDF/Xacro: PlanningScene, comparación RRTConnect/RRTstar, ciclo pick-and-place con perfiles cúbico/quíntico, validación DH y Jacobiano.

## Requisitos

- Ubuntu con ROS 2 Jazzy, MoveIt 2, RViz 2, ros2_control, `xacro`, `check_urdf`.
- Python 3 con NumPy y Matplotlib.
- MATLAB u Octave (opcional, sin Robotics System Toolbox).

## Estructura

```
UR5_MoveIt/
├── src/                        # paquetes ROS2 (ur5_description, ur5_moveit_config, ur5_pick_place)
├── matlab/                     # validación DH y generación de gráficas de evidencia
├── scripts/                    # utilidades para generar gráficas de evidencia
├── resultados/                 # ros2/, logs/, tablas/, imagenes/, matlab/, historico/
├── EVIDENCIA_COMPLETA_TALLER.md # evidencia principal, numeral por numeral
└── COMANDOS_RAPIDOS.md          # comandos para la sustentación
```

## Compilación

```bash
cd UR5_MoveIt
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## Ejecución

Con el workspace compilado y en dos terminales (ambas con `source install/setup.bash`):

```bash
# Terminal 1: MoveIt2 + RViz
QT_QPA_PLATFORM=xcb ros2 launch ur5_moveit_config demo.launch.py

# Terminal 2 (esperar a que la Terminal 1 diga "You can start planning now"):
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=FULL -p results_dir:=resultados/ros2
```

Ver [COMANDOS_RAPIDOS.md](COMANDOS_RAPIDOS.md) para el comando exacto de cada parte del taller (modelo, HOME, IK, PlanningScene, 4A-4D, Jacobiano, ciclo completo, MATLAB) y [EVIDENCIA_COMPLETA_TALLER.md](EVIDENCIA_COMPLETA_TALLER.md) para el resultado numérico real de cada una.

## Documentación

- [EVIDENCIA_COMPLETA_TALLER.md](EVIDENCIA_COMPLETA_TALLER.md) — evidencia principal, numeral por numeral.
- [COMANDOS_RAPIDOS.md](COMANDOS_RAPIDOS.md) — comandos para la sustentación.

Los resultados se guardan en `resultados/` (separados en `ros2/`, `logs/`, `tablas/`, `imagenes/`, `matlab/` e `historico/`).
