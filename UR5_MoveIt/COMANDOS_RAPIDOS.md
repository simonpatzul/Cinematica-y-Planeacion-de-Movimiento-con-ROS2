# Comandos rápidos de sustentación

Para todos los comandos de abajo: MoveIt2 + RViz deben estar activos (`QT_QPA_PLATFORM=xcb ros2 launch ur5_moveit_config demo.launch.py` en una terminal, esperar "You can start planning now") y en la terminal de trabajo hay que tener `source install/setup.bash` con `cd UR5_MoveIt` como directorio actual. Los resultados quedan en `resultados/ros2/`.

## Escena base (necesaria antes de cualquier prueba)

```bash
ros2 run ur5_pick_place planning_scene_setup
```

Aplica `pick_surface`, `place_surface`, `workpiece` y `obstacle`.

## Modelo UR5 y MoveIt

```bash
xacro src/ur5_description/urdf/ur5.urdf.xacro > /tmp/ur5_taller.urdf
check_urdf /tmp/ur5_taller.urdf
```

Revisar además `src/ur5_moveit_config/config/ur5.srdf` (grupo de planeación, estados nombrados, matriz de autocolisiones), `joint_limits.yaml`, `kinematics.yaml` (solver KDL) y `ompl_planning.yaml` (RRTConnect/RRTstar).

## HOME: DH vs ROS

```bash
ros2 run ur5_pick_place move_named_state --ros-args -p target:=HOME -p execute:=true
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=HOME -p results_dir:=resultados/ros2
```

Compara la transformación `base_link -> tool0` calculada por MoveIt/TF contra el modelo DH. Ver `resultados/ros2/kinematics_HOME.{txt,csv}`.

## IK PICK / PLACE

```bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=PICK -p results_dir:=resultados/ros2
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=PLACE -p results_dir:=resultados/ros2
```

Resuelve la IK de PICK y PLACE con MoveIt/KDL y verifica esas articulaciones contra el modelo DH.

## PlanningScene

```bash
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 24}}"
```

Confirma los 4 objetos aplicados por `planning_scene_setup`.

## RRTConnect vs RRTstar (HOME -> PRE-PICK)

```bash
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4A -p results_dir:=resultados/ros2
```

20 candidatos (10 por planeador), tabla comparativa en `resultados/ros2/trayectorias_4A_home_pre_pick.csv` y ejecución del ganador.

## Cúbico vs quíntico (PRE-PICK -> PICK)

```bash
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4B -p results_dir:=resultados/ros2
```

PRE-PICK -> PICK con `computeCartesianPath`, comparación de perfiles en `resultados/ros2/perfiles_4B_pre_pick_pick.csv` y adjuntado de la pieza.

## PICK -> PRE-PLACE

```bash
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4C -p results_dir:=resultados/ros2
```

Confirma la pieza adjunta y repite la comparación de planeadores con esa geometría extra.

## PLACE

```bash
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4D -p results_dir:=resultados/ros2
```

Reutiliza el perfil elegido en 4B, verifica límites de velocidad/aceleración y libera la pieza.

## Jacobiano

```bash
for estado in HOME PICK PLACE 4B 4D; do
  ros2 run ur5_pick_place kinematics_report --ros-args -p state:="$estado" -p results_dir:=resultados/ros2
done
```

J_DH vs J_MoveIt/KDL en los 5 estados, con verificación de `xdot = J·qdot` en `resultados/ros2/kinematics_{HOME,PICK,PLACE,4B,4D}.txt`.

## Ciclo completo

```bash
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=FULL -p results_dir:=resultados/ros2
```

HOME -> PRE-PICK -> PICK -> ATTACH -> PRE-PLACE -> PLACE -> DETACH.

## MATLAB

```bash
cd matlab
octave --no-gui --eval "UR5_DH_VALIDACION"
octave --no-gui --eval "generar_evidencia_grafica"
```

(o `matlab -batch "..."` si se tiene MATLAB con licencia). Corre la validación DH y genera las gráficas de evidencia a partir de los CSV/TXT reales en `../resultados/matlab/`.
