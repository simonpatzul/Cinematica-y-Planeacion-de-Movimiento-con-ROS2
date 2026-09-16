# Taller UR5 con ROS 2 Jazzy, MoveIt 2 y RViz 2

Este workspace contiene los tres paquetes fuente propios, el modelo cinemático
DH para MATLAB y una carpeta destinada a guardar los resultados del taller.
No incluye `build/`, `install/`, `log/` ni el repositorio oficial completo usado
como referencia.

## Estructura

```text
ur5_taller_ws/
├── README.md
├── matlab/
│   └── UR5_DH_HOME.m
├── src/
│   ├── ur5_description/
│   ├── ur5_moveit_config/
│   └── ur5_pick_place/
└── resultados/
```

## Instalación y compilación

Extraiga el ZIP de modo que los paquetes queden dentro de `ur5_taller_ws/src/`.
Después ejecute:

```bash
cd ~/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ur5_taller_ws/install/setup.bash
```

## Ejecución

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5_taller_ws/install/setup.bash
QT_QPA_PLATFORM=xcb ros2 launch ur5_moveit_config demo.launch.py
```

Terminal 2, después de que RViz termine de cargar:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5_taller_ws/install/setup.bash
ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true
ros2 run ur5_pick_place move_named_state --ros-args -p robot_description_kinematics.ur_manipulator.kinematics_solver:=kdl_kinematics_plugin/KDLKinematicsPlugin -p target:=HOME -p execute:=true
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p robot_description_kinematics.ur_manipulator.kinematics_solver:=kdl_kinematics_plugin/KDLKinematicsPlugin
```

Abra `matlab/UR5_DH_HOME.m` en MATLAB para calcular la cinemática directa de
la configuración HOME y comparar el esqueleto DH con el modelo 3D del UR5.
Guarde capturas, gráficas y demás evidencias en `resultados/`.
