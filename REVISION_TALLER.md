# Revisión punto por punto del taller UR5

Fecha de revisión: 2026-09-15  
Workspace revisado: `/home/simon/ur5_taller_ws`  
Respaldo consultado: `/home/simon/ur5_taller_ws_respaldo_20260915`

## 1. Alcance y criterio de revisión

Se comparó el workspace organizado con el respaldo y se revisaron:

- estructura y archivos entregables;
- paquetes ROS 2, dependencias y compilación;
- URDF/Xacro, MoveIt 2, ros2_control y controladores;
- escena de planificación y secuencia pick-and-place;
- script MATLAB de parámetros DH;
- documentación, resultados y reproducibilidad;
- pruebas que sí pueden ejecutarse en este entorno.

Los directorios `build/`, `install/` y `log/` del respaldo son artefactos de compilación, no código fuente. El repositorio oficial `Universal_Robots_ROS2_Description` también se conserva solamente en el respaldo porque el taller ya incluye su paquete reducido `ur5_description`.

## 2. Estructura solicitada

### Estado: CUMPLE

La estructura actual es:

```text
ur5_taller_ws/
├── README.md
├── REVISION_TALLER.md
├── matlab/
│   └── UR5_DH_HOME.m
├── src/
│   ├── ur5_description/
│   ├── ur5_moveit_config/
│   └── ur5_pick_place/
└── resultados/
    └── .gitkeep
```

Comprobación ejecutada:

```bash
cd /home/simon/ur5_taller_ws
find . -maxdepth 2 -mindepth 1 -printf '%y %p\n' | sort
```

Resultado relevante:

```text
./README.md
./matlab/UR5_DH_HOME.m
./src/ur5_description
./src/ur5_moveit_config
./src/ur5_pick_place
./resultados/.gitkeep
```

### Qué falta agregar

No falta ninguna carpeta estructural. La carpeta `resultados/` está vacía porque todavía no se han agregado capturas o evidencias del funcionamiento en RViz.

## 3. Paquete `ur5_description`

### Estado: CUMPLE

Contiene:

- `urdf/ur5.urdf.xacro`;
- macro URDF y archivos de control;
- parámetros físicos, visuales, cinemáticos y límites;
- mallas visuales y de colisión del UR5;
- launch de visualización;
- configuración RViz.

El Xacro usa `ur_type:=ur5`, incluye los parámetros propios del UR5 clásico y genera los enlaces hasta `tool0`.

Comando de revisión:

```bash
find src/ur5_description -type f | sort
```

Validación ejecutada después de instalar temporalmente el paquete:

```bash
source /opt/ros/jazzy/setup.bash
xacro /tmp/ur5_full_audit.*/install/ur5_description/share/ur5_description/urdf/ur5.urdf.xacro > /tmp/ur5_description.urdf
check_urdf /tmp/ur5_description.urdf
```

Resultado: URDF generado correctamente; la cadena termina en `tool0` y no hubo error de análisis.

### Qué conviene mejorar

- Cambiar la descripción `TODO: Package description` del `package.xml` por una descripción académica del taller.
- Cambiar `simon@todo.todo` por el correo institucional definitivo si el paquete será entregado públicamente.
- Añadir una sección de licencia/autoría coherente en el README.

## 4. Paquete `ur5_moveit_config`

### Estado: CUMPLE

Contiene:

- grupo de planificación `ur_manipulator` desde `base_link` hasta `tool0`;
- estados nombrados `HOME` y `READY`;
- configuración de cinemática KDL;
- límites articulares;
- control simulado `mock_components/GenericSystem`;
- `joint_trajectory_controller`;
- configuración de MoveIt Simple Controller Manager;
- launch `demo.launch.py` y archivos auxiliares;
- configuración RViz.

Comprobaciones ejecutadas:

```bash
xacro /tmp/ur5_full_audit.*/install/ur5_moveit_config/share/ur5_moveit_config/config/ur5.urdf.xacro > /tmp/ur5_moveit.urdf
check_urdf /tmp/ur5_moveit.urdf

source /tmp/ur5_full_audit.*/install/setup.bash
ros2 launch ur5_moveit_config demo.launch.py --show-args
```

Resultado:

- URDF de MoveIt generado correctamente.
- El grupo `ur_manipulator` está definido.
- El launch acepta sus argumentos (`use_rviz`, `db`, `debug`, etc.).

### Qué falta o debe quedar explícito

- El taller funciona en simulación; `FakeSystem` no controla un UR5 físico.
- Si se quiere hardware real, hay que agregar/configurar el driver de Universal Robots, IP del robot, calibración y una política de seguridad. No se debe ejecutar el código actual directamente sobre hardware sin esa adaptación.
- El taller no incluye una pinza física ni un actuador de gripper; `attachObject` solamente simula la sujeción en MoveIt.

## 5. Paquete `ur5_pick_place`

### Estado: CUMPLE EN CÓDIGO

Incluye cuatro ejecutables:

| Ejecutable | Función |
|---|---|
| `planning_scene_setup` | Crea o elimina mesa, pieza y obstáculo |
| `move_named_state` | Planifica y opcionalmente ejecuta `HOME` o `READY` |
| `plan_pre_pick` | Planifica `PRE-PICK`; no ejecuta por defecto |
| `pick_place_sequence` | Ejecuta la secuencia completa pick-and-place |

La escena utiliza el marco `base_link` y define:

- mesa: `0.90 x 1.20 x 0.20 m`, superficie superior en `z=0`;
- pieza: `0.05 x 0.05 x 0.07 m`, inicialmente en `(0.65, -0.30, 0.04)`;
- obstáculo: `0.16 x 0.08 x 0.40 m`, centrado en `(0.65, 0.0, 0.20)`.

La secuencia implementada es:

```text
HOME
READY
SUBIDA DESDE READY
DESPLAZAMIENTO SOBRE PICK
DESCENSO A PRE-PICK
PRE-PICK -> PICK
PICK: attachObject(workpiece, tool0)
ELEVACIÓN
SUBIDA SOBRE OBSTÁCULO
PASO SOBRE OBSTÁCULO
SALIDA SOBRE OBSTÁCULO
ENTRADA A PRE-PLACE
PRE-PLACE -> PLACE
PLACE: detachObject(workpiece)
```

### Riesgo técnico identificado

La pieza se adjunta a `tool0`, `flange` y `wrist_3_link` mediante `attachObject`. Esto es suficiente para una simulación de planificación, pero no representa una pinza ni una transformación física de TCP. Para una demostración física se debe agregar un gripper y su controlador.

## 6. Dependencias del sistema

### Estado: DISPONIBLES EN ESTE EQUIPO

Se verificó:

```bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
for p in moveit_ros_planning_interface controller_manager \\
         joint_trajectory_controller xacro rviz2; do
  ros2 pkg prefix "$p"
done
```

Resultado:

```text
jazzy
/opt/ros/jazzy   # moveit_ros_planning_interface
/opt/ros/jazzy   # controller_manager
/opt/ros/jazzy   # joint_trajectory_controller
/opt/ros/jazzy   # xacro
/opt/ros/jazzy   # rviz2
```

## 7. Compilación limpia

### Estado: APROBADA

Para no contaminar el workspace se compiló en `/tmp`:

```bash
source /opt/ros/jazzy/setup.bash
AUDIT=$(mktemp -d /tmp/ur5_full_audit.XXXXXX)
colcon --log-base "$AUDIT/log" build \\
  --base-paths src \\
  --build-base "$AUDIT/build" \\
  --install-base "$AUDIT/install"
```

Resultado observado:

```text
Finished <<< ur5_description
Finished <<< ur5_moveit_config
Finished <<< ur5_pick_place
Summary: 3 packages finished
```

La compilación muestra una advertencia de CMake sobre `tl_expected` obsoleto dentro de dependencias de MoveIt. No impide la compilación del taller y no procede del código fuente del paquete.

## 8. Validación de XML, Xacro, YAML y Python

### Estado: APROBADA CON UNA ACLARACIÓN

Se validaron los XML/Xacro y archivos YAML. Los Xacro generaron URDF después de instalar el paquete en el entorno temporal.

El archivo `src/ur5_description/config/ur5/joint_limits.yaml` contiene etiquetas `!degrees`. Un parser YAML genérico puede reportar:

```text
could not determine a constructor for the tag '!degrees'
```

Esto no es un defecto del taller: `!degrees` es una etiqueta interpretada por Xacro. El propio procesamiento Xacro terminó correctamente.

También se compiló la sintaxis Python de los launch:

```bash
python3 -m compileall -q src/ur5_moveit_config/launch
```

Resultado: correcto.

## 9. Arranque de MoveIt y controladores

### Estado: ARRANQUE COMPROBADO; EJECUCIÓN INTERACTIVA PENDIENTE

Se ejecutó:

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/ur5_full_audit.*/install/setup.bash
timeout --signal=INT 18s \\
  ros2 launch ur5_moveit_config demo.launch.py use_rviz:=false
```

El log mostró:

```text
Configured and activated ur5_arm_controller
Configured and activated joint_state_broadcaster
Successfully switched controllers!
```

La orden termina con código `124` porque se usa `timeout` deliberadamente para detener un launch que permanece activo. En el entorno aislado aparecen además mensajes `getifaddrs: Operation not permitted` y errores UDP de Fast DDS; son restricciones del sandbox para sockets, no errores del archivo de configuración.

En un equipo ROS 2 normal se debe dejar este launch activo en la Terminal 1 y no usar `timeout`.

## 10. Procedimiento de ejecución del taller

### Terminal 1: MoveIt y RViz

```bash
cd ~/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
QT_QPA_PLATFORM=xcb ros2 launch ur5_moveit_config demo.launch.py
```

Esperar hasta ver el UR5 en RViz y los controladores activos.

### Terminal 2: preparar la escena

```bash
cd ~/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true
ros2 run ur5_pick_place planning_scene_setup
```

### Terminal 2: pruebas individuales

```bash
ros2 run ur5_pick_place move_named_state \\
  --ros-args \\
  -p robot_description_kinematics.ur_manipulator.kinematics_solver:=kdl_kinematics_plugin/KDLKinematicsPlugin \\
  -p target:=HOME

ros2 run ur5_pick_place plan_pre_pick
```

### Terminal 2: secuencia completa

```bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence \\
  --ros-args \\
  -p robot_description_kinematics.ur_manipulator.kinematics_solver:=kdl_kinematics_plugin/KDLKinematicsPlugin
```

La ejecución completa no se marcó como aprobada en este entorno porque DDS no puede abrir sockets dentro del sandbox. Debe ejecutarse en una sesión ROS 2 local con comunicación DDS habilitada.

## 11. MATLAB y cinemática DH

### Estado: PRESENTE; EVIDENCIA VISUAL PENDIENTE

Está incluido:

```text
matlab/UR5_DH_HOME.m
```

El script contiene:

- configuración HOME de seis articulaciones;
- parámetros DH modificados del UR5 clásico;
- cinemática directa;
- conversión al marco `base_link`;
- gráfico del esqueleto DH;
- comparación con `loadrobot('universalUR5')` cuando Robotics System Toolbox está disponible.

Para ejecutarlo:

```matlab
cd('/home/simon/ur5_taller_ws/matlab');
UR5_DH_HOME
```

### Qué agregar

Guardar en `resultados/` una captura o exportación de:

- la transformación final `base_link -> tool0`;
- la posición de `tool0`;
- la figura “Esqueleto calculado con DH”;
- la comparación con el modelo 3D, si MATLAB dispone de Robotics System Toolbox.

## 12. Resultados y evidencias

### Estado: PENDIENTE DE COMPLETAR

Actualmente `resultados/` solo contiene `.gitkeep`. Para una entrega completa deben agregarse, como mínimo:

```text
resultados/
├── 01_compilacion.txt
├── 02_rviz_ur5.png
├── 03_escena_planificacion.png
├── 04_home_ready.png
├── 05_pick_place.png
└── 06_dh_home.png
```

Los nombres son sugeridos; pueden sustituirse por las evidencias reales. No se deben inventar capturas: deben obtenerse durante una ejecución local verificable.

## 13. Pruebas automatizadas

### Estado: FALTA AGREGAR

Los tres paquetes no tienen una batería de tests propia para la lógica del taller. Se recomienda agregar:

1. prueba de existencia y nombres de los estados `HOME` y `READY`;
2. prueba de dimensiones y posiciones de `table`, `workpiece` y `obstacle`;
3. prueba de que los ejecutables se instalan;
4. prueba de generación del URDF con Xacro;
5. prueba de que los launch y YAML se pueden cargar.

Como mínimo, el informe final debe conservar el resultado de:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

Ejecución realizada durante esta revisión:

```text
Summary: 3 packages finished
Summary: 0 tests, 0 errors, 0 failures, 0 skipped
```

El código de salida fue `0`; el valor `0 tests` confirma que todavía no hay
tests propios registrados, no que se hayan cubierto los casos de la lógica.

## 14. Calidad de entrega y documentación

### Estado: PARCIAL

Ya están presentes `README.md` y este informe. Falta decidir si se desea conservar también la documentación MATLAB anterior del respaldo (`documentacion_proyecto_ur5.m`). Es útil como guía de procedimiento, pero no pertenece a la estructura mínima solicitada. Si se conserva, la ubicación recomendada sería:

```text
matlab/documentacion_proyecto_ur5.m
```

También se recomienda reemplazar los metadatos `TODO` de `ur5_description` y `ur5_pick_place` antes de entregar.

## 15. Resumen final

| Punto | Estado | Acción necesaria |
|---|---|---|
| Estructura del workspace | CUMPLE | Ninguna |
| Tres paquetes ROS 2 | CUMPLE | Ninguna |
| URDF/Xacro | CUMPLE | Validado después de compilar |
| MoveIt 2 | CUMPLE | Validar visualmente en RViz local |
| Controladores simulados | CUMPLE | Ninguna para simulación |
| Escena de planificación | CUMPLE EN CÓDIGO | Capturar evidencia en RViz |
| Pick-and-place | CUMPLE EN CÓDIGO | Ejecutar en DDS local y guardar log/capturas |
| MATLAB DH | CUMPLE | Guardar resultados gráficos |
| `resultados/` | PENDIENTE | Agregar evidencias reales |
| Tests propios | FALTA | Agregar pruebas automatizadas |
| Hardware real/gripper | FUERA DEL ALCANCE ACTUAL | Requiere integración adicional |
| Metadatos de paquetes | MEJORABLE | Completar descripción, autor y correo |

## Conclusión

El taller está técnicamente completo para una demostración simulada: compila, genera el URDF y arranca la infraestructura de MoveIt 2 y ros2_control. Lo que falta para una entrega académica cerrada son las evidencias de ejecución en RViz/MATLAB, los tests propios y la limpieza de metadatos. La prueba interactiva de ROS 2 debe hacerse fuera de este sandbox porque la comunicación DDS está restringida.
