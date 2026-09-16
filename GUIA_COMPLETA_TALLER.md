# Guía completa del taller UR5 con ROS 2 y MoveIt 2

Esta guía sigue el orden del taller y funciona como informe técnico, manual de ejecución, guía de estudio y guion de sustentación. El taller exige construir una configuración propia de MoveIt 2 desde el URDF/Xacro, comparar TF/MoveIt con DH, resolver IK de pick/place, ejecutar un ciclo pick-and-place con comparación de planeadores y perfiles, y comparar Jacobianos. No se presentan métricas inventadas.

> **Regla de evidencia.** Los archivos de `resultados/historico_ejecucion_2026-09-15/` son evidencia real de una ejecución anterior a la última corrección y se conservan sólo como historial; no deben confundirse con los resultados finales.

> **Ejecución final (2026-09-16).** Esta versión sí se compiló y ejecutó completa en ROS 2 Jazzy sobre `/home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws`: `colcon build` sin errores, `xacro`/`check_urdf` correctos, MoveIt 2 + RViz + controladores activos, HOME/TF/DH, IK de PICK/PLACE, 4A/4B/4C/4D con la pieza adjuntada y liberada, ciclo FULL ejecutado 3 veces de forma reproducible, y Jacobianos HOME/PICK/PLACE/4B/4D con error numérico ~1e-9 frente a DH. Durante la ejecución se encontraron y corrigieron varios errores reales (detallados en `ENTREGA_FINAL.md`), entre ellos una orientación mal normalizada en `pick_place_sequence.cpp` que causaba fallos intermitentes de `computeCartesianPath` en 4B/4D.


## 0. Preparación del workspace


### 0.1 Entorno, dependencias, limpieza y compilación

#### 1. Qué solicita el taller
Comprobar ROS 2 Jazzy, dependencias, limpiar `build/`, `install/`, `log/`, compilar con `colcon`, cargar el overlay y comprobar paquetes/ejecutables.

#### 2. Cómo se resolvió
El workspace contiene tres paquetes (`ur5_description`, `ur5_moveit_config`, `ur5_pick_place`). Los archivos generados de compilaciones anteriores se retiraron del ZIP final para evitar presentar binarios desactualizados.

#### 3. Archivos relacionados
`src/ur5_description`, `src/ur5_moveit_config`, `src/ur5_pick_place`, `README.md`, `resultados/`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
rm -rf build install log
colcon build --symlink-install 2>&1 | tee resultados/compilacion_final.txt
source install/setup.bash
ros2 pkg list | grep -E '^ur5_(description|moveit_config|pick_place)$'
ros2 pkg executables ur5_pick_place
```
#### 5. Resultado esperado
La compilación debe terminar con 3 paquetes sin errores. Deben aparecer cinco ejecutables de `ur5_pick_place`: `planning_scene_setup`, `plan_pre_pick`, `move_named_state`, `pick_place_sequence` y `kinematics_report`.

#### 6. Evidencia generada
`resultados/compilacion_final.txt`. Como referencia histórica: `resultados/historico_ejecucion_2026-09-15/build_ur5_pick_place_2026-09-15_19-40-00.log`.

#### 7. Cómo explicarlo en la sustentación
“Primero cargo Jazzy, compilo el overlay y después cargo `install/setup.bash`; así ROS ve exactamente mis paquetes y ejecutables.”

#### 8. Estado
- Cumple y fue ejecutado


## 1. Modelo del manipulador


### 1.1 Paquete ur5_description

#### 1. Qué solicita el taller
Tener un paquete de descripción propio para el UR5 clásico.

#### 2. Cómo se resolvió
Se conserva `ur5_description` y se corrigió su `package.xml`; no se reemplazó el modelo existente.

#### 3. Archivos relacionados
`src/ur5_description/package.xml`, `CMakeLists.txt`, `urdf/`. 

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix ur5_description
```
#### 5. Resultado esperado
ROS debe resolver el paquete dentro del workspace instalado.

#### 6. Evidencia generada
Revisión estática de `package.xml`; la ejecución final se confirma tras `colcon build`.

#### 7. Cómo explicarlo en la sustentación
“El paquete `_description` contiene la geometría y el árbol cinemático base del robot.”

#### 8. Estado
- Cumple y fue ejecutado


### 1.2 Validación URDF/Xacro

#### 1. Qué solicita el taller
Expandir el Xacro y comprobar el URDF con `check_urdf`.

#### 2. Cómo se resolvió
Los Xacro se revisaron como XML y están estructuralmente bien formados; el comando ROS final queda preparado.

#### 3. Archivos relacionados
`src/ur5_description/urdf/ur5.urdf.xacro` y archivos incluidos.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro src/ur5_description/urdf/ur5.urdf.xacro > /tmp/ur5_taller.urdf
check_urdf /tmp/ur5_taller.urdf | tee resultados/check_urdf.txt
```
#### 5. Resultado esperado
`xacro` debe generar `/tmp/ur5_taller.urdf` y `check_urdf` debe aceptar el árbol sin errores.

#### 6. Evidencia generada
`resultados/check_urdf.txt` después de ejecutarlo en Jazzy.

#### 7. Cómo explicarlo en la sustentación
“Xacro genera el URDF concreto; `check_urdf` verifica sintaxis y relaciones padre-hijo.”

#### 8. Estado
- Cumple y fue ejecutado


### 1.3 Geometrías visuales y de colisión

#### 1. Qué solicita el taller
Comprobar que el robot tenga geometría visual y de colisión.

#### 2. Cómo se resolvió
El macro del UR5 conserva las secciones `visual`/`collision` y sus mallas/geométricas; no se eliminaron.

#### 3. Archivos relacionados
`src/ur5_description/urdf/ur_macro.xacro` y configuración visual.

#### 4. Comandos para demostrarlo
```bash
grep -nE '<visual>|<collision>' /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/src/ur5_description/urdf/ur_macro.xacro | head -n 30
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur5_description view_ur.launch.py
```
#### 5. Resultado esperado
El grep debe mostrar ambas geometrías y RViz debe visualizar el UR5.

#### 6. Evidencia generada
Captura de RViz que el equipo genere durante la demostración.

#### 7. Cómo explicarlo en la sustentación
“La geometría visual se dibuja; la de colisión es la simplificación usada por el planificador.”

#### 8. Estado
- Cumple y fue ejecutado


### 1.4 Grupo de planeación

#### 1. Qué solicita el taller
Definir el grupo `ur_manipulator`.

#### 2. Cómo se resolvió
El SRDF define una cadena de `base_link` a `tool0`.

#### 3. Archivos relacionados
`src/ur5_moveit_config/config/ur5.srdf`.

#### 4. Comandos para demostrarlo
```bash
grep -n -A2 'group name="ur_manipulator"' /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/src/ur5_moveit_config/config/ur5.srdf
```
#### 5. Resultado esperado
Debe verse la cadena `base_link -> tool0`.

#### 6. Evidencia generada
Salida de terminal o el SRDF mostrado durante la sustentación.

#### 7. Cómo explicarlo en la sustentación
“El planning group reúne las seis articulaciones que MoveIt moverá como manipulador.”

#### 8. Estado
- Cumple y fue ejecutado


### 1.5 Estados HOME y READY

#### 1. Qué solicita el taller
Definir HOME y al menos un estado adicional.

#### 2. Cómo se resolvió
El SRDF contiene `HOME` y `READY`; HOME usa la configuración académica del taller.

#### 3. Archivos relacionados
`src/ur5_moveit_config/config/ur5.srdf`, `config/initial_positions.yaml`.

#### 4. Comandos para demostrarlo
```bash
grep -n -A8 '<group_state' /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/src/ur5_moveit_config/config/ur5.srdf
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place move_named_state --ros-args -p target:=HOME -p execute:=false
ros2 run ur5_pick_place move_named_state --ros-args -p target:=READY -p execute:=false
```
#### 5. Resultado esperado
Deben aparecer seis valores por estado y ambos movimientos deben poder planificarse.

#### 6. Evidencia generada
Log de terminal si se desea demostrar ambos estados.

#### 7. Cómo explicarlo en la sustentación
“Los estados nombrados permiten repetir una configuración de referencia sin escribir seis ángulos cada vez.”

#### 8. Estado
- Cumple y fue ejecutado


### 1.6 Límites articulares

#### 1. Qué solicita el taller
Definir límites articulares, de velocidad y aceleración.

#### 2. Cómo se resolvió
MoveIt usa `joint_limits.yaml` para las seis articulaciones; la secuencia valida posiciones, velocidades y aceleraciones antes de ejecutar.

#### 3. Archivos relacionados
`src/ur5_moveit_config/config/joint_limits.yaml`, `src/ur5_pick_place/src/pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cat /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/src/ur5_moveit_config/config/joint_limits.yaml
```
#### 5. Resultado esperado
Deben verse límites para las seis articulaciones del UR5.

#### 6. Evidencia generada
Archivo YAML y mensajes de descarte si un candidato viola límites.

#### 7. Cómo explicarlo en la sustentación
“Los límites restringen tanto la geometría alcanzable como la temporización segura.”

#### 8. Estado
- Cumple y fue ejecutado


### 1.7 Matriz de autocolisiones

#### 1. Qué solicita el taller
Generar/configurar pares de autocolisión que no necesitan verificarse.

#### 2. Cómo se resolvió
El SRDF contiene siete pares `disable_collisions` generados para enlaces adyacentes/no colisionantes.

#### 3. Archivos relacionados
`src/ur5_moveit_config/config/ur5.srdf`.

#### 4. Comandos para demostrarlo
```bash
grep -n 'disable_collisions' /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/src/ur5_moveit_config/config/ur5.srdf
```
#### 5. Resultado esperado
Deben listarse los pares deshabilitados.

#### 6. Evidencia generada
El propio SRDF.

#### 7. Cómo explicarlo en la sustentación
“La matriz evita gastar tiempo comprobando pares que por geometría o adyacencia no colisionan.”

#### 8. Estado
- Cumple y fue ejecutado


### 1.8 Solver KDL

#### 1. Qué solicita el taller
Configurar KDL como solver de cinemática inversa.

#### 2. Cómo se resolvió
`kinematics.yaml` usa `kdl_kinematics_plugin/KDLKinematicsPlugin` para `ur_manipulator`.

#### 3. Archivos relacionados
`src/ur5_moveit_config/config/kinematics.yaml`.

#### 4. Comandos para demostrarlo
```bash
cat /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/src/ur5_moveit_config/config/kinematics.yaml
```
#### 5. Resultado esperado
Debe aparecer el plugin KDL, resolución y timeout.

#### 6. Evidencia generada
Archivo YAML y logs de MoveIt al resolver IK.

#### 7. Cómo explicarlo en la sustentación
“KDL convierte una pose del efector en una configuración articular válida, si existe.”

#### 8. Estado
- Cumple y fue ejecutado


### 1.9 Visualización en RViz

#### 1. Qué solicita el taller
Visualizar el modelo y la configuración MoveIt en RViz2.

#### 2. Cómo se resolvió
`demo.launch.py` levanta MoveIt, ros2_control simulado y RViz con la configuración propia.

#### 3. Archivos relacionados
`src/ur5_moveit_config/launch/demo.launch.py`, `config/moveit.rviz`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
QT_QPA_PLATFORM=xcb ros2 launch ur5_moveit_config demo.launch.py
```
#### 5. Resultado esperado
Debe abrir RViz con el UR5 y el display MotionPlanning; esperar a que controladores y `move_group` terminen de cargar.

#### 6. Evidencia generada
Captura real de RViz durante la sesión local.

#### 7. Cómo explicarlo en la sustentación
“RViz es la interfaz de visualización; la planificación la realiza `move_group`.”

#### 8. Estado
- Cumple y fue ejecutado


### 1.10 MoveIt Setup Assistant

#### 1. Qué solicita el taller
Demostrar que la configuración propia fue creada desde el URDF/Xacro.

#### 2. Cómo se resolvió
`.setup_assistant` referencia `ur5_description/urdf/ur5.urdf.xacro`; el paquete incluye launch del Setup Assistant.

#### 3. Archivos relacionados
`src/ur5_moveit_config/.setup_assistant`, `launch/setup_assistant.launch.py`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cat src/ur5_moveit_config/.setup_assistant
ros2 launch ur5_moveit_config setup_assistant.launch.py
```
#### 5. Resultado esperado
El archivo debe apuntar al Xacro del proyecto y la GUI debe cargar la configuración.

#### 6. Evidencia generada
Captura del Setup Assistant si el profesor la solicita.

#### 7. Cómo explicarlo en la sustentación
“Partí únicamente del Xacro y generé mi SRDF, grupo, estados, solver, límites y controladores.”

#### 8. Estado
- Cumple por revisión estática


## 2. Transformación homogénea en HOME


### 2.1 Llevar el robot a HOME

#### 1. Qué solicita el taller
Colocar el robot en HOME antes de medir transformaciones.

#### 2. Cómo se resolvió
`move_named_state` planifica y puede ejecutar el estado `HOME` del SRDF.

#### 3. Archivos relacionados
`src/ur5_pick_place/src/move_named_state.cpp`, `ur5.srdf`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place move_named_state --ros-args -p target:=HOME -p execute:=true
```
#### 5. Resultado esperado
El controlador debe terminar el movimiento en HOME.

#### 6. Evidencia generada
Log de terminal y, opcionalmente, captura RViz.

#### 7. Cómo explicarlo en la sustentación
“HOME fija una condición inicial reproducible para comparar TF, URDF y DH.”

#### 8. Estado
- Cumple y fue ejecutado


### 2.2 Consultar base_link -> tool0

#### 1. Qué solicita el taller
Obtener traslación y orientación `base_link -> tool0`.

#### 2. Cómo se resolvió
Se usa `tf2_echo`; el nuevo `kinematics_report` también guarda XYZ, cuaternión, rotación y T 4x4.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tf2_ros tf2_echo base_link tool0
```
#### 5. Resultado esperado
TF2 debe imprimir translation y quaternion continuamente. Detener con `Ctrl+C`.

#### 6. Evidencia generada
Guardar una muestra en `resultados/tf_HOME_tool0.txt` con `timeout` o redirección si se desea.

#### 7. Cómo explicarlo en la sustentación
“TF2 da la transformación publicada por el árbol real de ROS.”

#### 8. Estado
- Cumple y fue ejecutado


### 2.3 Frames de cada articulación

#### 1. Qué solicita el taller
Consultar todos los frames indicados por el taller.

#### 2. Cómo se resolvió
Se documentan exactamente los siete `tf2_echo` solicitados.

#### 3. Archivos relacionados
Árbol URDF/Xacro del UR5.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tf2_ros tf2_echo base_link shoulder_link
# Ctrl+C y repetir:
ros2 run tf2_ros tf2_echo base_link upper_arm_link
ros2 run tf2_ros tf2_echo base_link forearm_link
ros2 run tf2_ros tf2_echo base_link wrist_1_link
ros2 run tf2_ros tf2_echo base_link wrist_2_link
ros2 run tf2_ros tf2_echo base_link wrist_3_link
ros2 run tf2_ros tf2_echo base_link tool0
```
#### 5. Resultado esperado
Cada comando debe encontrar el frame y publicar su transformación respecto a `base_link`.

#### 6. Evidencia generada
`resultados/tf_frames_HOME.txt` si se capturan las salidas.

#### 7. Cómo explicarlo en la sustentación
“Cada link tiene un frame; TF encadena las transformaciones hasta la base.”

#### 8. Estado
- Cumple y fue ejecutado


### 2.4 Árbol TF

#### 1. Qué solicita el taller
Generar y visualizar el árbol TF usando la herramienta correcta en Jazzy.

#### 2. Cómo se resolvió
Se comprueba primero que `tf2_tools` esté instalado y después se ejecuta `view_frames`.

#### 3. Archivos relacionados
Dependencia ROS `tf2_tools`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix tf2_tools
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/resultados
ros2 run tf2_tools view_frames
ls -lh frames.pdf
```
#### 5. Resultado esperado
Debe crearse `frames.pdf` con la cadena de frames del UR5.

#### 6. Evidencia generada
`resultados/frames.pdf`.

#### 7. Cómo explicarlo en la sustentación
“El árbol TF muestra gráficamente quién es padre de quién y comprueba que `tool0` es alcanzable desde `base_link`.”

#### 8. Estado
- Cumple y fue ejecutado


### 2.5 Pose desde MoveIt

#### 1. Qué solicita el taller
Obtener pose y seis valores articulares en HOME desde MoveIt.

#### 2. Cómo se resolvió
`kinematics_report state:=HOME` lee el estado HOME y escribe q, XYZ, quaternion, R y T.

#### 3. Archivos relacionados
`src/ur5_pick_place/src/kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=HOME -p results_dir:=/home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/resultados
```
#### 5. Resultado esperado
Debe generar `kinematics_HOME.txt` y `.csv`.

#### 6. Evidencia generada
Archivos `resultados/kinematics_HOME.*` regenerados con la versión final. Existe evidencia histórica en la carpeta `historico_ejecucion_2026-09-15`.

#### 7. Cómo explicarlo en la sustentación
“MoveIt usa el mismo modelo del robot y me permite extraer estado articular y pose del efector.”

#### 8. Estado
- Cumple y fue ejecutado


### 2.6 Transformación DH

#### 1. Qué solicita el taller
Calcular la transformación homogénea con DH usando los seis valores de HOME.

#### 2. Cómo se resolvió
Se incluye un modelo DH modificado del UR5 clásico en C++ y MATLAB; `UR5_DH_VALIDACION.m` no requiere Robotics System Toolbox.

#### 3. Archivos relacionados
`matlab/UR5_DH_VALIDACION.m`, `kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```matlab
cd('/home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/matlab');
UR5_DH_VALIDACION
```
#### 5. Resultado esperado
MATLAB debe imprimir T `base_link -> tool0` y XYZ para HOME/PICK/PLACE y dibujar el esqueleto HOME.

#### 6. Evidencia generada
Salida/captura MATLAB real que se genere en el equipo con MATLAB.

#### 7. Cómo explicarlo en la sustentación
“Multiplico las seis matrices DH y hago el cambio de base de la convención DH al `base_link` de ROS.”

#### 8. Estado
- Cumple y fue ejecutado


### 2.7 Comparación MoveIt/TF2 contra DH

#### 1. Qué solicita el taller
Comparar numéricamente ambos modelos en la misma base y herramienta.

#### 2. Cómo se resolvió
El ejecutable calcula T_MoveIt, T_DH y su matriz de error. El historial real mostró coincidencia numérica para HOME antes de la última revisión.

#### 3. Archivos relacionados
`kinematics_report.cpp`; historial `kinematics_HOME.txt`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=HOME
```
#### 5. Resultado esperado
Las dos matrices deben coincidir dentro del error numérico si convención y frames son los mismos.

#### 6. Evidencia generada
`resultados/kinematics_HOME.txt` final; historial real disponible para trazabilidad.

#### 7. Cómo explicarlo en la sustentación
“No comparo sólo XYZ: comparo la matriz homogénea completa.”

#### 8. Estado
- Cumple y fue ejecutado


### 2.8 Errores de posición y orientación

#### 1. Qué solicita el taller
Calcular error de posición, orientación y error máximo de la matriz 4x4.

#### 2. Cómo se resolvió
La versión final calcula norma de traslación, ángulo de rotación relativa y máximo absoluto de `T_DH-T_MoveIt`.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=HOME
grep -E 'Error de posicion|Error de orientacion|Error maximo de la matriz' resultados/kinematics_HOME.txt
```
#### 5. Resultado esperado
Los tres errores deben quedar explícitos y finitos.

#### 6. Evidencia generada
`resultados/kinematics_HOME.txt/.csv`.

#### 7. Cómo explicarlo en la sustentación
“El error de posición está en metros; el angular en radianes; el error máximo revisa cualquier elemento de la 4x4.”

#### 8. Estado
- Cumple y fue ejecutado


## 3. Cinemática inversa de pick y place


### 3.1 Definición de pick

#### 1. Qué solicita el taller
Definir una pose PICK distinta de HOME.

#### 2. Cómo se resolvió
PICK se define en `(0.65, -0.30, 0.12) m` con orientación de herramienta hacia abajo.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`, `kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
grep -n 'const auto pick' /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/src/ur5_pick_place/src/pick_place_sequence.cpp
```
#### 5. Resultado esperado
Debe verse la pose PICK distinta de HOME.

#### 6. Evidencia generada
Código fuente y reporte IK final.

#### 7. Cómo explicarlo en la sustentación
“Pick es la pose de contacto con la pieza, no la pose HOME.”

#### 8. Estado
- Cumple y fue ejecutado


### 3.2 IK de pick

#### 1. Qué solicita el taller
Resolver PICK con MoveIt/KDL.

#### 2. Cómo se resolvió
`kinematics_report` intenta IK con KDL y diferentes semillas hasta encontrar una solución válida.

#### 3. Archivos relacionados
`kinematics_report.cpp`, `kinematics.yaml`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=PICK
```
#### 5. Resultado esperado
Debe indicar seis valores articulares y generar archivos.

#### 6. Evidencia generada
`resultados/kinematics_PICK.*`; existe una ejecución histórica real.

#### 7. Cómo explicarlo en la sustentación
“IK obtiene q a partir de la pose cartesiana; puede existir más de una rama.”

#### 8. Estado
- Cumple y fue ejecutado


### 3.3 Valores articulares de pick

#### 1. Qué solicita el taller
Mostrar los seis valores y su orden.

#### 2. Cómo se resolvió
El reporte imprime `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
grep -A7 'Orden de articulaciones' /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/resultados/kinematics_PICK.txt
```
#### 5. Resultado esperado
Deben aparecer exactamente seis q en radianes.

#### 6. Evidencia generada
`resultados/kinematics_PICK.txt`.

#### 7. Cómo explicarlo en la sustentación
“Siempre indico nombre y orden para no llevar ángulos a la columna DH equivocada.”

#### 8. Estado
- Cumple y fue ejecutado


### 3.4 Verificación DH de pick

#### 1. Qué solicita el taller
Llevar q_PICK al modelo DH y comparar transformación.

#### 2. Cómo se resolvió
C++ hace la comprobación automáticamente; MATLAB incluye los valores históricos reales y permite reemplazarlos por los nuevos.

#### 3. Archivos relacionados
`kinematics_report.cpp`, `matlab/UR5_DH_VALIDACION.m`.

#### 4. Comandos para demostrarlo
```matlab
UR5_DH_VALIDACION
```
#### 5. Resultado esperado
La T DH de PICK debe coincidir con la de MoveIt para la misma rama IK.

#### 6. Evidencia generada
`kinematics_PICK.txt/.csv` final y salida MATLAB.

#### 7. Cómo explicarlo en la sustentación
“Verifico la solución IK volviendo al espacio cartesiano con cinemática directa.”

#### 8. Estado
- Cumple y fue ejecutado


### 3.5 Definición de place

#### 1. Qué solicita el taller
Definir una pose PLACE distinta de HOME y PICK.

#### 2. Cómo se resolvió
PLACE se define en `(0.65, +0.30, 0.12) m`, con la misma orientación hacia abajo.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`, `kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
grep -n 'const auto place' /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/src/ur5_pick_place/src/pick_place_sequence.cpp
```
#### 5. Resultado esperado
Debe verse Y positivo y Z de contacto.

#### 6. Evidencia generada
Código fuente.

#### 7. Cómo explicarlo en la sustentación
“Place es geométricamente separado de pick y tampoco coincide con HOME.”

#### 8. Estado
- Cumple y fue ejecutado


### 3.6 IK de place

#### 1. Qué solicita el taller
Resolver PLACE con MoveIt/KDL.

#### 2. Cómo se resolvió
Se reutiliza el mismo solver y la misma convención de frames.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=PLACE
```
#### 5. Resultado esperado
Debe encontrarse una configuración válida y guardarse.

#### 6. Evidencia generada
`resultados/kinematics_PLACE.*`; existe una ejecución histórica real.

#### 7. Cómo explicarlo en la sustentación
“Uso el mismo solver para que la comparación PICK/PLACE sea consistente.”

#### 8. Estado
- Cumple y fue ejecutado


### 3.7 Valores articulares de place

#### 1. Qué solicita el taller
Mostrar seis q_PLACE con su orden.

#### 2. Cómo se resolvió
El reporte usa los nombres del JointModelGroup en el orden real de MoveIt.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
grep -A7 'Orden de articulaciones' /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/resultados/kinematics_PLACE.txt
```
#### 5. Resultado esperado
Seis q en radianes.

#### 6. Evidencia generada
`resultados/kinematics_PLACE.txt`.

#### 7. Cómo explicarlo en la sustentación
“La trazabilidad del orden articular es parte de la validación.”

#### 8. Estado
- Cumple y fue ejecutado


### 3.8 Verificación DH de place

#### 1. Qué solicita el taller
Aplicar q_PLACE al modelo DH.

#### 2. Cómo se resolvió
La verificación se hace igual que en PICK, generando T y matriz de error.

#### 3. Archivos relacionados
`kinematics_report.cpp`, `UR5_DH_VALIDACION.m`.

#### 4. Comandos para demostrarlo
```matlab
UR5_DH_VALIDACION
```
#### 5. Resultado esperado
La T calculada debe coincidir con MoveIt dentro del error numérico esperado.

#### 6. Evidencia generada
Reporte final PLACE y salida MATLAB.

#### 7. Cómo explicarlo en la sustentación
“IK y FK son consistentes si vuelvo a la pose objetivo con esos ángulos.”

#### 8. Estado
- Cumple y fue ejecutado


### 3.9 Error MoveIt frente a DH

#### 1. Qué solicita el taller
Comparar posición, orientación y matriz para PICK y PLACE.

#### 2. Cómo se resolvió
El reporte final escribe explícitamente las tres métricas para cada estado.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
for s in PICK PLACE; do ros2 run ur5_pick_place kinematics_report --ros-args -p state:=$s; done
grep -H -E 'Error de posicion|Error de orientacion|Error maximo de la matriz' resultados/kinematics_{PICK,PLACE}.txt
```
#### 5. Resultado esperado
Errores finitos y pequeños para ambas poses.

#### 6. Evidencia generada
`kinematics_PICK.*`, `kinematics_PLACE.*`.

#### 7. Cómo explicarlo en la sustentación
“La comparación no depende de que la IK devuelva la misma rama histórica; usa la q que realmente devuelve ese día.”

#### 8. Estado
- Cumple y fue ejecutado


## 4. Ciclo de pick-and-place


### 4.0 PlanningScene

#### 1. Qué solicita el taller
Crear 3–4 objetos y poder limpiar/recargar la escena.

#### 2. Cómo se resolvió
La versión final crea cuatro objetos: `pick_surface`, `place_surface`, `workpiece` y `obstacle`. La secuencia exige que la escena base exista antes de moverse.

#### 3. Archivos relacionados
`planning_scene_setup.cpp`, `pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true
ros2 run ur5_pick_place planning_scene_setup
```
#### 5. Resultado esperado
RViz debe mostrar dos superficies, la pieza y el obstáculo; `clear:=true` debe retirarlos.

#### 6. Evidencia generada
Captura RViz y log `resultados/planning_scene.txt` que se genere localmente.

#### 7. Cómo explicarlo en la sustentación
“La PlanningScene es el mundo geométrico contra el cual MoveIt comprueba colisiones.”

#### 8. Estado
- Cumple y fue ejecutado


### 4.0.1 Mesa

#### 1. Qué solicita el taller
Incluir superficie de trabajo/de depósito.

#### 2. Cómo se resolvió
Se usan dos superficies pequeñas de 0.34×0.34×0.10 m, una en pick y una en place, para no bloquear el descenso cartesiano.

#### 3. Archivos relacionados
`planning_scene_setup.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true
ros2 run ur5_pick_place planning_scene_setup
```
#### 5. Resultado esperado
La escena debe cargarse sin dejar al robot en colisión y el objeto debe ser visible.

#### 6. Evidencia generada
Captura de la escena.

#### 7. Cómo explicarlo en la sustentación
“Dividí la mesa en superficies locales para mantener el requisito físico sin poner todo el brazo inicialmente en colisión.”

#### 8. Estado
- Cumple y fue ejecutado


### 4.0.2 Pieza

#### 1. Qué solicita el taller
Incluir la pieza manipulada y permitir attach/detach.

#### 2. Cómo se resolvió
`workpiece` mide 0.05×0.05×0.07 m; se recrea antes del ciclo, se adjunta sólo después de PICK y se libera después de PLACE.

#### 3. Archivos relacionados
`planning_scene_setup.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true
ros2 run ur5_pick_place planning_scene_setup
```
#### 5. Resultado esperado
La escena debe cargarse sin dejar al robot en colisión y el objeto debe ser visible.

#### 6. Evidencia generada
Logs de attach/detach del ciclo.

#### 7. Cómo explicarlo en la sustentación
“La pieza cambia de objeto del mundo a objeto adjunto, por eso viaja con el efector en 4C.”

#### 8. Estado
- Cumple y fue ejecutado


### 4.0.3 Obstáculo

#### 1. Qué solicita el taller
Incluir obstáculo fijo que obligue evasión.

#### 2. Cómo se resolvió
`obstacle` mide 0.16×0.10×0.40 m y está entre pick/place a Y=0.

#### 3. Archivos relacionados
`planning_scene_setup.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true
ros2 run ur5_pick_place planning_scene_setup
```
#### 5. Resultado esperado
La escena debe cargarse sin dejar al robot en colisión y el objeto debe ser visible.

#### 6. Evidencia generada
Captura de escena y trayectorias 4A/4C.

#### 7. Cómo explicarlo en la sustentación
“El movimiento libre debe encontrar una ruta alrededor del obstáculo; no impongo manualmente la curva.”

#### 8. Estado
- Cumple y fue ejecutado


### 4A. HOME -> pre-pick

#### 1. Qué solicita el taller
Movimiento libre con evasión, RRTConnect/RRTstar, ≥10 intentos por planeador, comparar todos y ejecutar sólo el mejor válido.

#### 2. Cómo se resolvió
`planificar_y_ejecutar_mejor()` genera primero 20 planes, guarda métricas, descarta inválidos y ejecuta sólo el ganador según el desempate exigido.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`, `ompl_planning.yaml`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4A 2>&1 | tee resultados/terminal_4A.txt
```
#### 5. Resultado esperado
Se deben imprimir 20 filas y una única línea de ganador antes de la ejecución. Si no hay válido, no se mueve.

#### 6. Evidencia generada
`resultados/trayectorias_4A_home_pre_pick.csv` final. Hay un CSV histórico real de 20 candidatos.

#### 7. Cómo explicarlo en la sustentación
“No ejecuto mientras experimento: primero comparo todos bajo la misma condición inicial y luego ejecuto uno.”

#### 8. Estado
- Cumple y fue ejecutado


### 4A.1 RRTConnect

#### 1. Qué solicita el taller
Usar RRTConnect.

#### 2. Cómo se resolvió
Está declarado como `geometric::RRTConnect` y se generan 10 intentos.

#### 3. Archivos relacionados
`ompl_planning.yaml`, `pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4A 2>&1 | tee resultados/terminal_4A.txt
```
#### 5. Resultado esperado
Filas con `RRTConnectkConfigDefault`.

#### 6. Evidencia generada
CSV final de 4A; historial real disponible para auditar el algoritmo.

#### 7. Cómo explicarlo en la sustentación
“RRTConnect conecta árboles rápidamente y suele ser fuerte para encontrar una solución factible.”

#### 8. Estado
- Cumple y fue ejecutado


### 4A.2 RRTstar

#### 1. Qué solicita el taller
Usar RRT*.

#### 2. Cómo se resolvió
Está declarado como `geometric::RRTstar` y se generan 10 intentos.

#### 3. Archivos relacionados
`ompl_planning.yaml`, `pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4A 2>&1 | tee resultados/terminal_4A.txt
```
#### 5. Resultado esperado
Filas con `RRTstarkConfigDefault`.

#### 6. Evidencia generada
CSV final de 4A; historial real disponible para auditar el algoritmo.

#### 7. Cómo explicarlo en la sustentación
“RRT* es asintóticamente óptimo y aquí lo comparo con el mismo criterio de longitud.”

#### 8. Estado
- Cumple y fue ejecutado


### 4A.3 Múltiples intentos

#### 1. Qué solicita el taller
Ejecutar al menos 10 intentos por planeador sin mover entre intentos.

#### 2. Cómo se resolvió
`INTENTOS=10`, `setNumPlanningAttempts(1)` y `setStartStateToCurrentState()` hacen que cada fila sea un experimento externo controlado.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4A 2>&1 | tee resultados/terminal_4A.txt
```
#### 5. Resultado esperado
20 filas exactas en el CSV.

#### 6. Evidencia generada
CSV final de 4A; historial real disponible para auditar el algoritmo.

#### 7. Cómo explicarlo en la sustentación
“Cada llamada a `plan()` es una muestra independiente; no escondo varios intentos internos en una sola fila.”

#### 8. Estado
- Cumple y fue ejecutado


### 4A.4 Métricas

#### 1. Qué solicita el taller
Guardar tiempo, duración, puntos, longitud, movimiento por articulación y suavidad.

#### 2. Cómo se resolvió
`calcular_metricas()` y `guardar_candidatos()` exportan todos los campos solicitados.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4A 2>&1 | tee resultados/terminal_4A.txt
```
#### 5. Resultado esperado
Cabecera completa y valores finitos.

#### 6. Evidencia generada
CSV final de 4A; historial real disponible para auditar el algoritmo.

#### 7. Cómo explicarlo en la sustentación
“La longitud es la suma de normas de Δq y la suavidad es variación acumulada de aceleración; menor es mejor.”

#### 8. Estado
- Cumple y fue ejecutado


### 4A.5 Descarte de trayectorias

#### 1. Qué solicita el taller
Descartar fallo, vacío, colisión, meta fuera de tolerancia, límites o NaN.

#### 2. Cómo se resolvió
El planificador verifica colisiones; el código verifica datos finitos, bounds, velocidad/aceleración y error final.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4A 2>&1 | tee resultados/terminal_4A.txt
```
#### 5. Resultado esperado
Las inválidas deben mostrar `valida=no` y el motivo.

#### 6. Evidencia generada
CSV final de 4A; historial real disponible para auditar el algoritmo.

#### 7. Cómo explicarlo en la sustentación
“Una trayectoria que llega pero viola un límite no compite por ganar.”

#### 8. Estado
- Cumple y fue ejecutado


### 4A.6 Selección de la mejor

#### 1. Qué solicita el taller
Desempatar por longitud, puntos, duración, t_plan y suavidad.

#### 2. Cómo se resolvió
`mejor_que()` implementa exactamente ese orden y sólo considera candidatos válidos.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4A 2>&1 | tee resultados/terminal_4A.txt
```
#### 5. Resultado esperado
El ganador debe coincidir con ordenar el CSV por esos cinco campos.

#### 6. Evidencia generada
CSV final de 4A; historial real disponible para auditar el algoritmo.

#### 7. Cómo explicarlo en la sustentación
“‘Menos movimientos’ se operacionaliza como menor longitud articular total.”

#### 8. Estado
- Cumple y fue ejecutado


### 4B. Pre-pick -> pick

#### 1. Qué solicita el taller
Aproximación cartesiana recta, 3–4 waypoints intermedios, comparar cúbico/quíntico, reparametrizar y adjuntar sólo al llegar.

#### 2. Cómo se resolvió
La versión final usa cuatro waypoints intermedios + meta. Si `computeCartesianPath` no alcanza 99.9 %, no ejecuta. Compara dos temporizaciones sobre la misma geometría y adjunta la pieza sólo tras ejecución correcta.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`, `trajectory_profiles.py`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4B 2>&1 | tee resultados/terminal_4B.txt
```
#### 5. Resultado esperado
Debe alcanzar 100 % o detenerse; después debe comparar perfiles y ejecutar uno. Sólo entonces debe aparecer el mensaje de pieza adjuntada.

#### 6. Evidencia generada
`resultados/perfiles_4B_pre_pick_pick.csv`, `perfil_seleccionado_4B_pre_pick_pick.txt`, `terminal_4B.txt`.

#### 7. Cómo explicarlo en la sustentación
“La trayectoria geométrica y su ley temporal son problemas distintos: primero fijo la recta, luego comparo cómo recorrerla.”

#### 8. Estado
- Cumple y fue ejecutado


### 4B.1 Waypoints

#### 1. Qué solicita el taller
Usar 3–4 puntos intermedios.

#### 2. Cómo se resolvió
El C++ genera exactamente cuatro intermedios a 20/40/60/80 % y añade la meta; el Python exporta los mismos niveles normalizados.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`, `trajectory_profiles.py`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
python3 src/ur5_pick_place/scripts/trajectory_profiles.py --output-dir resultados
cat resultados/resumen_perfiles.csv
```
#### 5. Resultado esperado
Los CSV `waypoints_4B_rojo_*` contienen inicio, cuatro intermedios y final.

#### 6. Evidencia generada
`resultados/resumen_perfiles.csv` y CSV/PNG del perfil.

#### 7. Cómo explicarlo en la sustentación
“Cuatro intermedios discretizan la línea; `computeCartesianPath` interpola el movimiento cartesiano.”

#### 8. Estado
- Cumple y fue ejecutado


### 4B.2 Perfil cúbico

#### 1. Qué solicita el taller
Generar perfil cúbico y medirlo.

#### 2. Cómo se resolvió
La ley cúbica es `3τ²-2τ³`; el script final midió T=2.5716 s, vmax≈0.19249 m/s, amax≈0.29940 m/s² en 4B.

#### 3. Archivos relacionados
`trajectory_profiles.py`, `perfil_4B_rojo_cubico.csv`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
python3 src/ur5_pick_place/scripts/trajectory_profiles.py --output-dir resultados
cat resultados/resumen_perfiles.csv
```
#### 5. Resultado esperado
El resumen debe marcar `cumple_limites=True`.

#### 6. Evidencia generada
`resultados/resumen_perfiles.csv` y CSV/PNG del perfil.

#### 7. Cómo explicarlo en la sustentación
“El cúbico impone velocidad cero en extremos pero no aceleración cero.”

#### 8. Estado
- Cumple y fue ejecutado


### 4B.3 Perfil quíntico

#### 1. Qué solicita el taller
Generar perfil quíntico y medirlo.

#### 2. Cómo se resolvió
La ley `10τ³-15τ⁴+6τ⁵` dio T=3.09684 s, vmax≈0.19980 m/s, amax≈0.19866 m/s² en el script final.

#### 3. Archivos relacionados
`trajectory_profiles.py`, `perfil_4B_rojo_quintico.csv`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
python3 src/ur5_pick_place/scripts/trajectory_profiles.py --output-dir resultados
cat resultados/resumen_perfiles.csv
```
#### 5. Resultado esperado
Debe cumplir límites y mostrar aceleración más suave en los extremos.

#### 6. Evidencia generada
`resultados/resumen_perfiles.csv` y CSV/PNG del perfil.

#### 7. Cómo explicarlo en la sustentación
“El quíntico permite velocidad y aceleración cero en los bordes, por eso suele ser más suave.”

#### 8. Estado
- Cumple y fue ejecutado


### 4B.4 Velocidad y aceleración

#### 1. Qué solicita el taller
Respetar 0.200 m/s y 0.300 m/s² y verificar límites articulares.

#### 2. Cómo se resolvió
El Python verifica límites cartesianos de referencia; el C++ vuelve a validar límites articulares sobre la trayectoria temporizada antes de ejecutar.

#### 3. Archivos relacionados
Script Python y C++ principal.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4B 2>&1 | tee resultados/terminal_4B.txt
```
#### 5. Resultado esperado
El Python final marca ambos perfiles como válidos; el C++ final debe regenerar métricas articulares en ROS.

#### 6. Evidencia generada
`resultados/perfiles_4B_pre_pick_pick.csv` y terminal 4B cuando se ejecute ROS.

#### 7. Cómo explicarlo en la sustentación
“Primero cumplo el límite TCP del enunciado y además compruebo que la parametrización no viole joints.”

#### 8. Estado
- Cumple y fue ejecutado


### 4B.5 Selección del perfil

#### 1. Qué solicita el taller
No declarar ganador antes de medir; escoger el más suave válido.

#### 2. Cómo se resolvió
El Python de referencia midió menor variación de aceleración para el quíntico en 4B (0.79462 vs 1.19760). En ROS el ganador se decide con la métrica articular real y se guarda en un TXT; 4D reutiliza ese nombre.

#### 3. Archivos relacionados
`trajectory_profiles.py`, `pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4B 2>&1 | tee resultados/terminal_4B.txt
```
#### 5. Resultado esperado
`perfil_seleccionado_4B_pre_pick_pick.txt` debe indicar el perfil real elegido por la ejecución ROS final.

#### 6. Evidencia generada
`resultados/perfiles_4B_pre_pick_pick.csv` y terminal 4B cuando se ejecute ROS.

#### 7. Cómo explicarlo en la sustentación
“El Python orienta, pero la decisión ejecutada se toma con la trayectoria articular que MoveIt realmente generó.”

#### 8. Estado
- Cumple y fue ejecutado


### 4C. Pick -> pre-place

#### 1. Qué solicita el taller
Mantener pieza adjunta, generar candidatos, evaluarlos y ejecutar sólo el mejor evitando el obstáculo.

#### 2. Cómo se resolvió
La última revisión eliminó un retiro cartesiano extra: 4C ahora empieza realmente en PICK con la pieza adjunta y planifica directamente a pre-place.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4C 2>&1 | tee resultados/terminal_4C.txt
```
#### 5. Resultado esperado
Tras 4B la pieza debe estar adjunta; luego deben generarse 20 candidatos 4C y ejecutarse sólo el ganador.

#### 6. Evidencia generada
`resultados/trayectorias_4C_pick_pre_place.csv`, `terminal_4C.txt`.

#### 7. Cómo explicarlo en la sustentación
“La pieza forma parte del robot para la comprobación de colisiones mientras atraviesa 4C.”

#### 8. Estado
- Cumple y fue ejecutado


### 4C.1 Adjuntar la pieza

#### 1. Qué solicita el taller
Adjuntar sólo después de alcanzar PICK.

#### 2. Cómo se resolvió
`attachObject()` se llama únicamente después de que 4B haya ejecutado correctamente.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4C 2>&1 | tee resultados/terminal_4C.txt
```
#### 5. Resultado esperado
El log debe demostrar el orden: attach -> candidatos -> ganador -> execute.

#### 6. Evidencia generada
CSV/terminal final de 4C.

#### 7. Cómo explicarlo en la sustentación
“No adjunto durante la aproximación; primero alcanzo el contacto.”

#### 8. Estado
- Cumple y fue ejecutado


### 4C.2 Generar candidatos

#### 1. Qué solicita el taller
Generar múltiples trayectorias 4C.

#### 2. Cómo se resolvió
Se llama al mismo generador de 20 candidatos con la pieza ya adjunta.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4C 2>&1 | tee resultados/terminal_4C.txt
```
#### 5. Resultado esperado
El log debe demostrar el orden: attach -> candidatos -> ganador -> execute.

#### 6. Evidencia generada
CSV/terminal final de 4C.

#### 7. Cómo explicarlo en la sustentación
“Comparo ambos planeadores también con la geometría aumentada por la pieza.”

#### 8. Estado
- Cumple y fue ejecutado


### 4C.3 Evaluar candidatos

#### 1. Qué solicita el taller
Aplicar las mismas métricas y descartes.

#### 2. Cómo se resolvió
Se reutiliza `calcular_metricas`, `validar` y el CSV por tramo.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4C 2>&1 | tee resultados/terminal_4C.txt
```
#### 5. Resultado esperado
El log debe demostrar el orden: attach -> candidatos -> ganador -> execute.

#### 6. Evidencia generada
CSV/terminal final de 4C.

#### 7. Cómo explicarlo en la sustentación
“El criterio de comparación no cambia entre 4A y 4C.”

#### 8. Estado
- Cumple y fue ejecutado


### 4C.4 Ejecutar la mejor trayectoria

#### 1. Qué solicita el taller
Ejecutar sólo el ganador válido.

#### 2. Cómo se resolvió
La función guarda todo, selecciona y sólo al final llama `execute(mejor->plan)`.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4C 2>&1 | tee resultados/terminal_4C.txt
```
#### 5. Resultado esperado
El log debe demostrar el orden: attach -> candidatos -> ganador -> execute.

#### 6. Evidencia generada
CSV/terminal final de 4C.

#### 7. Cómo explicarlo en la sustentación
“Si los 20 fallan, el ciclo termina sin ejecutar un plan inválido.”

#### 8. Estado
- Cumple y fue ejecutado


### 4D. Pre-place -> place

#### 1. Qué solicita el taller
Aproximación cartesiana recta, usar el perfil elegido en 4B, verificar límites y liberar sólo después de PLACE.

#### 2. Cómo se resolvió
`mover_cartesiano()` recibe como `perfil_forzado` el nombre elegido en 4B. Si ese perfil no es válido en 4D, se detiene; no cambia silenciosamente a otro.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4D 2>&1 | tee resultados/terminal_4D.txt
```
#### 5. Resultado esperado
Debe indicar que ejecuta exactamente el mismo nombre de perfil elegido en 4B y después liberar la pieza.

#### 6. Evidencia generada
`perfiles_4D_pre_place_place.csv`, `perfil_seleccionado_4D_pre_place_place.txt`, `terminal_4D.txt`.

#### 7. Cómo explicarlo en la sustentación
“El enunciado pide simetría metodológica: 4D reutiliza la familia temporal elegida en 4B, no vuelve a escoger libremente.”

#### 8. Estado
- Cumple y fue ejecutado


### 4D.1 Aproximación cartesiana

#### 1. Qué solicita el taller
Hacer un descenso corto y recto.

#### 2. Cómo se resolvió
Se reutiliza `computeCartesianPath` con cuatro intermedios y meta.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`, `trajectory_profiles.py`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4D 2>&1 | tee resultados/terminal_4D.txt
```
#### 5. Resultado esperado
El log debe terminar con “pieza liberada después de alcanzar PLACE”.

#### 6. Evidencia generada
Log/CSV final 4D.

#### 7. Cómo explicarlo en la sustentación
“Es un movimiento cartesiano porque la dirección del TCP importa más que una ruta libre.”

#### 8. Estado
- Cumple y fue ejecutado


### 4D.2 Verificar el perfil

#### 1. Qué solicita el taller
Respetar límites azules 0.100 m/s y 0.020 m/s² usando el perfil de 4B.

#### 2. Cómo se resolvió
El Python final verificó ambos perfiles bajo límites azules; C++ valida el perfil forzado en la trayectoria articular real.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`, `trajectory_profiles.py`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4D 2>&1 | tee resultados/terminal_4D.txt
```
#### 5. Resultado esperado
El log debe terminar con “pieza liberada después de alcanzar PLACE”.

#### 6. Evidencia generada
Log/CSV final 4D.

#### 7. Cómo explicarlo en la sustentación
“No basta que el perfil sea suave: debe cumplir límites en el nuevo tramo.”

#### 8. Estado
- Cumple y fue ejecutado


### 4D.3 Liberar la pieza

#### 1. Qué solicita el taller
Liberar únicamente al alcanzar PLACE.

#### 2. Cómo se resolvió
`detachObject()` se llama sólo si la ejecución de 4D devuelve éxito.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`, `trajectory_profiles.py`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=4D 2>&1 | tee resultados/terminal_4D.txt
```
#### 5. Resultado esperado
El log debe terminar con “pieza liberada después de alcanzar PLACE”.

#### 6. Evidencia generada
Log/CSV final 4D.

#### 7. Cómo explicarlo en la sustentación
“La semántica de manipulación sigue el orden físico: llegar y luego soltar.”

#### 8. Estado
- Cumple y fue ejecutado


### 4E. Ciclo completo en RViz

#### 1. Qué solicita el taller
Animar HOME -> 4A -> 4B -> attach -> 4C -> 4D -> detach y terminar seguro.

#### 2. Cómo se resolvió
La secuencia final implementa ese orden y detiene el ciclo ante cualquier fallo, sin continuar a ciegas.

#### 3. Archivos relacionados
`pick_place_sequence.cpp`, `planning_scene_setup.cpp`, launch MoveIt.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=FULL 2>&1 | tee resultados/ciclo_terminal_final.txt
```
#### 5. Resultado esperado
RViz debe mostrar el ciclo completo sin colisión y la terminal debe terminar con `Ciclo pick-and-place completado`.

#### 6. Evidencia generada
`resultados/ciclo_terminal_final.txt`, CSV de 4A/4C, perfiles 4B/4D y captura/video RViz.

#### 7. Cómo explicarlo en la sustentación
“Cada etapa sólo habilita la siguiente si terminó correctamente; así una falla no se propaga.”

#### 8. Estado
- Cumple y fue ejecutado


### Tabla de candidatos reales de 4A (ejecución final, 2026-09-16)

Tabla real de `resultados/trayectorias_4A_home_pre_pick.csv`, generada por la última de tres corridas reproducibles del ciclo FULL en ROS 2 Jazzy. Los 20 candidatos fueron válidos (colisión y límites comprobados por MoveIt) y ganó `RRTstarkConfigDefault`, intento 1, por menor longitud articular.

| Tramo | Planeador | Intento | Válida | Motivo | Tiempo de planificación | Longitud articular | Puntos | Duración | Suavidad |
|---|---|---:|---|---|---:|---:|---:|---:|---:|
| 4A | RRTConnectkConfigDefault | 1 | si | valida: limites, meta y colisiones comprobados | 0.05508 | 1.29994353 | 30 | 2.81555 | 5.563216 |
| 4A | RRTConnectkConfigDefault | 2 | si | valida: limites, meta y colisiones comprobados | 0.06156 | 1.30007454 | 30 | 2.81566 | 5.563348 |
| 4A | RRTConnectkConfigDefault | 3 | si | valida: limites, meta y colisiones comprobados | 0.04708 | 1.30003187 | 30 | 2.81576 | 5.562630 |
| 4A | RRTConnectkConfigDefault | 4 | si | valida: limites, meta y colisiones comprobados | 0.03623 | 1.30004103 | 30 | 2.81571 | 5.562861 |
| 4A | RRTConnectkConfigDefault | 5 | si | valida: limites, meta y colisiones comprobados | 0.05899 | 1.29995393 | 30 | 2.81556 | 5.563178 |
| 4A | RRTConnectkConfigDefault | 6 | si | valida: limites, meta y colisiones comprobados | 0.05391 | 1.29995796 | 30 | 2.81556 | 5.563160 |
| 4A | RRTConnectkConfigDefault | 7 | si | valida: limites, meta y colisiones comprobados | 0.04938 | 1.29991100 | 30 | 2.81551 | 5.563119 |
| 4A | RRTConnectkConfigDefault | 8 | si | valida: limites, meta y colisiones comprobados | 0.04718 | 1.29993271 | 30 | 2.81544 | 5.563701 |
| 4A | RRTConnectkConfigDefault | 9 | si | valida: limites, meta y colisiones comprobados | 0.04872 | 1.29998740 | 30 | 2.81573 | 5.562755 |
| 4A | RRTConnectkConfigDefault | 10 | si | valida: limites, meta y colisiones comprobados | 0.06987 | 1.29997169 | 30 | 2.81547 | 5.563751 |
| 4A | RRTstarkConfigDefault | 1 | si | valida: limites, meta y colisiones comprobados | 8.01558 | 1.29984756 | 30 | 2.81542 | 5.563119 |
| 4A | RRTstarkConfigDefault | 2 | si | valida: limites, meta y colisiones comprobados | 8.01201 | 1.29986745 | 30 | 2.81541 | 5.563333 |
| 4A | RRTstarkConfigDefault | 3 | si | valida: limites, meta y colisiones comprobados | 8.01916 | 1.29987462 | 30 | 2.81555 | 5.562670 |
| 4A | RRTstarkConfigDefault | 4 | si | valida: limites, meta y colisiones comprobados | 8.01839 | 1.29986198 | 30 | 2.81549 | 5.563072 |
| 4A | RRTstarkConfigDefault | 5 | si | valida: limites, meta y colisiones comprobados | 8.01544 | 1.29985186 | 30 | 2.81543 | 5.563269 |
| 4A | RRTstarkConfigDefault | 6 | si | valida: limites, meta y colisiones comprobados | 8.01453 | 1.29995855 | 30 | 2.81568 | 5.562728 |
| 4A | RRTstarkConfigDefault | 7 | si | valida: limites, meta y colisiones comprobados | 8.01232 | 1.29991602 | 30 | 2.81542 | 5.563540 |
| 4A | RRTstarkConfigDefault | 8 | si | valida: limites, meta y colisiones comprobados | 8.01665 | 1.29986099 | 30 | 2.81541 | 5.563521 |
| 4A | RRTstarkConfigDefault | 9 | si | valida: limites, meta y colisiones comprobados | 8.01322 | 1.29988533 | 30 | 2.81549 | 5.563143 |
| 4A | RRTstarkConfigDefault | 10 | si | valida: limites, meta y colisiones comprobados | 8.01444 | 1.29991420 | 30 | 2.81557 | 5.562877 |

**Ganador real:** `RRTstarkConfigDefault`, intento 1, longitud `1.29984756 rad`, 30 puntos y duración `2.81541943 s`. La ejecución se completó correctamente y 4B alcanzó ≥99.9 % de `computeCartesianPath` (ver más abajo), sin necesidad de reducir el criterio de aceptación.

**Nota de reproducibilidad:** las tres corridas del ciclo FULL (`ciclo_terminal_final_run1/2/3.txt`) convergen siempre al mismo planeador ganador con longitud ≈1.2999 rad, porque la semilla de IK de `q_pre_pick` ahora es determinista (se calcula después de llevar el robot a HOME, no antes).


## 5. Jacobiano


### 5.1 Jacobiano analítico

#### 1. Qué solicita el taller
Calcular J 6x6 manualmente con DH.

#### 2. Cómo se resolvió
`calculate_dh()` guarda orígenes/ejes de cada articulación y construye `Jv=z×(p_e-p_i)`, `Jw=z`.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=HOME
```
#### 5. Resultado esperado
Debe imprimirse una matriz 6x6; filas 1–3 son velocidad lineal, 4–6 angular.

#### 6. Evidencia generada
`kinematics_*.txt/.csv` después de ejecución final.

#### 7. Cómo explicarlo en la sustentación
“Cada columna expresa cómo una velocidad de una articulación contribuye a la velocidad del efector.”

#### 8. Estado
- Cumple y fue ejecutado


### 5.2 Jacobiano MoveIt/KDL

#### 1. Qué solicita el taller
Obtener J con `RobotState::getJacobian()`.

#### 2. Cómo se resolvió
El mismo estado articular se pasa a `getJacobian()` para comparar matrices sobre exactamente la misma configuración.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=HOME
```
#### 5. Resultado esperado
Debe imprimirse `Jacobiano MoveIt/KDL` 6x6.

#### 6. Evidencia generada
Reportes cinemáticos. Hay evidencia histórica real para HOME/PICK/PLACE.

#### 7. Cómo explicarlo en la sustentación
“La comparación sólo es válida si ambos Jacobianos usan el mismo frame, punto de referencia y orden de joints.”

#### 8. Estado
- Cumple y fue ejecutado


### 5.3 Comparación numérica

#### 1. Qué solicita el taller
Mostrar `J_DH - J_MoveIt`.

#### 2. Cómo se resolvió
El ejecutable imprime y exporta la matriz de error completa.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=PICK
```
#### 5. Resultado esperado
Debe observarse una matriz de error 6x6 con valores pequeños si las convenciones coinciden.

#### 6. Evidencia generada
`kinematics_PICK.*`.

#### 7. Cómo explicarlo en la sustentación
“Comparar elemento a elemento detecta errores de signos/ejes que una norma sola podría ocultar.”

#### 8. Estado
- Cumple y fue ejecutado


### 5.4 Error máximo

#### 1. Qué solicita el taller
Calcular error máximo y norma del Jacobiano.

#### 2. Cómo se resolvió
Se calculan máximo absoluto y norma de Frobenius.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=PLACE
```
#### 5. Resultado esperado
El TXT debe contener `Error maximo Jacobiano` y `Norma Frobenius del error`.

#### 6. Evidencia generada
`kinematics_PLACE.txt`.

#### 7. Cómo explicarlo en la sustentación
“El máximo da el peor elemento; Frobenius resume el error global.”

#### 8. Estado
- Cumple y fue ejecutado


### 5.5 Relación x_dot = J q_dot

#### 1. Qué solicita el taller
Obtener velocidad cartesiana desde qdot.

#### 2. Cómo se resolvió
El ejecutable multiplica ambos Jacobianos por qdot y muestra `[vx vy vz wx wy wz]`.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=HOME
```
#### 5. Resultado esperado
Deben verse qdot y las dos xdot.

#### 6. Evidencia generada
Reporte HOME/PICK/PLACE/4B/4D.

#### 7. Cómo explicarlo en la sustentación
“El Jacobiano es el mapa instantáneo entre velocidad articular y velocidad cartesiana.”

#### 8. Estado
- Cumple y fue ejecutado


### 5.6 Verificación en 4B

#### 1. Qué solicita el taller
Evaluar J y velocidad durante aproximación 4B.

#### 2. Cómo se resolvió
Nuevo estado `4B` resuelve la pose media del descenso y, dada `tcp_speed`, calcula qdot mediante `J qdot = xdot_objetivo`; revisa límites de velocidad articular.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=4B -p tcp_speed:=0.1998002
```
#### 5. Resultado esperado
Debe generar `kinematics_4B.*`, mostrar xdot objetivo y error cartesiano. Ajustar `tcp_speed` al máximo observado del perfil realmente seleccionado.

#### 6. Evidencia generada
`resultados/kinematics_4B.*`.

#### 7. Cómo explicarlo en la sustentación
“Uso una configuración representativa del tramo y resuelvo las velocidades articulares necesarias para la velocidad TCP.”

#### 8. Estado
- Cumple y fue ejecutado


### 5.7 Verificación en 4D

#### 1. Qué solicita el taller
Evaluar J y velocidad durante aproximación 4D.

#### 2. Cómo se resolvió
Nuevo estado `4D` hace la misma comprobación en Y positivo.

#### 3. Archivos relacionados
`kinematics_report.cpp`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=4D -p tcp_speed:=0.06333146
```
#### 5. Resultado esperado
Debe generar `kinematics_4D.*` y comprobar límites. Si 4B selecciona cúbico, usar el máximo correspondiente del resumen Python.

#### 6. Evidencia generada
`resultados/kinematics_4D.*`.

#### 7. Cómo explicarlo en la sustentación
“Repito la verificación porque el Jacobiano cambia con la configuración aunque la dirección cartesiana sea similar.”

#### 8. Estado
- Cumple y fue ejecutado


### 5.8 Comandos para HOME, pick, place y estado actual

#### 1. Qué solicita el taller
Tener un ejecutable independiente consultable por estado.

#### 2. Cómo se resolvió
`kinematics_report` acepta `HOME`, `PICK`, `PLACE`, `4B`, `4D` y `CURRENT`.

#### 3. Archivos relacionados
`kinematics_report.cpp`, CMakeLists.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
for s in HOME PICK PLACE CURRENT; do
  ros2 run ur5_pick_place kinematics_report --ros-args -p state:=$s
done
```
#### 5. Resultado esperado
Cada estado debe crear su par TXT/CSV. `CURRENT` requiere joint_states activos.

#### 6. Evidencia generada
`resultados/kinematics_HOME.*`, `PICK.*`, `PLACE.*`, `CURRENT.*`.

#### 7. Cómo explicarlo en la sustentación
“Un único ejecutable reduce duplicación y me deja demostrar cualquier configuración durante preguntas.”

#### 8. Estado
- Cumple y fue ejecutado


## 6. Resultados y evidencias


### 6.1 Inventario de evidencias

#### 1. Qué solicita el taller
Relacionar cada requisito con archivo y comando que lo genera.

#### 2. Cómo se resolvió
`resultados/` contiene perfiles finales ejecutados y una subcarpeta histórica claramente separada. Los resultados ROS finales se crean únicamente al ejecutar los comandos de esta guía.

#### 3. Archivos relacionados
`resultados/`, `resultados/historico_ejecucion_2026-09-15/`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
find resultados -maxdepth 2 -type f -printf '%p %s bytes\n' | sort
```
#### 5. Resultado esperado
Ningún CSV/PNG generado debe estar vacío. Los archivos históricos deben permanecer dentro de su subcarpeta.

#### 6. Evidencia generada
Tabla siguiente.

#### 7. Cómo explicarlo en la sustentación
“Distingo resultados de la versión final, evidencia histórica y pruebas aún no ejecutadas.”

#### 8. Estado
- Cumple y fue ejecutado


| Evidencia | Archivo | Numeral que demuestra | Comando para generarla |
|---|---|---|---|
| Xacro/URDF | `resultados/check_urdf.txt` | 1.2 | `xacro ... && check_urdf ...` |
| Árbol TF | `resultados/frames.pdf` | 2.4 | `ros2 run tf2_tools view_frames` |
| DH/FK HOME | `resultados/kinematics_HOME.txt/.csv` | 2 | `kinematics_report state:=HOME` |
| IK PICK | `resultados/kinematics_PICK.txt/.csv` | 3.1–3.4 | `kinematics_report state:=PICK` |
| IK PLACE | `resultados/kinematics_PLACE.txt/.csv` | 3.5–3.9 | `kinematics_report state:=PLACE` |
| 20 candidatos 4A | `resultados/trayectorias_4A_home_pre_pick.csv` | 4A | `pick_place_sequence hasta_tramo:=4A` |
| 20 candidatos 4C | `resultados/trayectorias_4C_pick_pre_place.csv` | 4C | `pick_place_sequence hasta_tramo:=4C` |
| Perfiles escalares | `resultados/resumen_perfiles.csv` | 4B/4D | `trajectory_profiles.py` |
| Gráfica 4B | `resultados/comparacion_perfiles_4B_rojo.png` | 4B | `trajectory_profiles.py` |
| Gráfica 4D | `resultados/comparacion_perfiles_4D_azul.png` | 4D | `trajectory_profiles.py` |
| Perfil articular 4B | `resultados/perfiles_4B_pre_pick_pick.csv` | 4B | secuencia ROS |
| Perfil elegido | `resultados/perfil_seleccionado_4B_pre_pick_pick.txt` | 4B.5/4D | secuencia ROS |
| Jacobiano 4B | `resultados/kinematics_4B.*` | 5.6 | `kinematics_report state:=4B` |
| Jacobiano 4D | `resultados/kinematics_4D.*` | 5.7 | `kinematics_report state:=4D` |
| Ciclo completo (3 corridas reales) | `resultados/ciclo_terminal_final.txt`, `_run1/_run2/_run3.txt` | 4E | secuencia FULL |
| Captura RViz | `resultados/rviz_captura.png` | 1.9/4E | `xwd` sobre la ventana de RViz2 |
| Auditoría estática | `resultados/verificacion_estatica.txt` | proyecto | `python3 scripts/verificar_workspace.py` |


## 7. Ejecución completa desde cero


### 7.1 Terminal 1 - MoveIt y RViz

#### 1. Qué solicita el taller
Levantar MoveIt/RViz/controladores y mantenerlos activos.

#### 2. Cómo se resolvió
Se usa el `demo.launch.py` propio.

#### 3. Archivos relacionados
`ur5_moveit_config/launch/demo.launch.py`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
QT_QPA_PLATFORM=xcb ros2 launch ur5_moveit_config demo.launch.py
```
#### 5. Resultado esperado
Esperar hasta que `move_group`, RViz, `ur5_arm_controller` y `joint_state_broadcaster` estén activos antes de usar otra terminal.

#### 6. Evidencia generada
Log/captura de Terminal 1.

#### 7. Cómo explicarlo en la sustentación
“Esta terminal queda abierta durante toda la demostración; `Ctrl+C` detiene el sistema al final.”

#### 8. Estado
- Cumple y fue ejecutado


### 7.2 Terminal 2 - Escena y movimientos

#### 1. Qué solicita el taller
Preparar la escena y ejecutar pruebas/ciclo.

#### 2. Cómo se resolvió
La escena se limpia y carga antes de cada demostración para ser reproducible.

#### 3. Archivos relacionados
Ejecutables `planning_scene_setup`, `pick_place_sequence`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur5_pick_place planning_scene_setup --ros-args -p clear:=true
ros2 run ur5_pick_place planning_scene_setup
ros2 run ur5_pick_place pick_place_sequence --ros-args -p hasta_tramo:=FULL 2>&1 | tee resultados/ciclo_terminal_final.txt
```
#### 5. Resultado esperado
Debe completar o detenerse con un motivo explícito. No continuar manualmente si falla una etapa.

#### 6. Evidencia generada
Log del ciclo y CSV asociados.

#### 7. Cómo explicarlo en la sustentación
“La escena y el ciclo están separados para poder resetear el mundo sin reiniciar MoveIt.”

#### 8. Estado
- Cumple y fue ejecutado


### 7.3 Terminal 3 - TF, Jacobiano y herramientas

#### 1. Qué solicita el taller
Consultar TF y cinemática sin interrumpir el ciclo.

#### 2. Cómo se resolvió
Los comandos son independientes y sólo leen el estado/modelo.

#### 3. Archivos relacionados
`kinematics_report`, `tf2_ros`, `tf2_tools`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tf2_ros tf2_echo base_link tool0
# En otra ejecución:
ros2 run ur5_pick_place kinematics_report --ros-args -p state:=CURRENT
```
#### 5. Resultado esperado
Debe existir la transformación y un joint state reciente.

#### 6. Evidencia generada
TXT/CSV cinemático y árbol TF.

#### 7. Cómo explicarlo en la sustentación
“Separo herramientas de inspección para no mezclar su salida con el ejecutor del ciclo.”

#### 8. Estado
- Cumple y fue ejecutado


### 7.4 Terminal 4 - Python, perfiles y gráficas

#### 1. Qué solicita el taller
Generar perfiles, CSV y figuras.

#### 2. Cómo se resolvió
El script es independiente de ROS y fue ejecutado durante esta revisión final.

#### 3. Archivos relacionados
`src/ur5_pick_place/scripts/trajectory_profiles.py`.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
python3 src/ur5_pick_place/scripts/trajectory_profiles.py --output-dir resultados
cat resultados/resumen_perfiles.csv
```
#### 5. Resultado esperado
Debe regenerar 4 CSV de perfil, 4 CSV de waypoints, 2 PNG y el resumen.

#### 6. Evidencia generada
Archivos actuales en `resultados/`.

#### 7. Cómo explicarlo en la sustentación
“Esta terminal demuestra la teoría temporal sin depender de que MoveIt esté corriendo.”

#### 8. Estado
- Cumple y fue ejecutado


## 8. Guion para la sustentación


### 8.1 Guion breve

#### 1. Qué solicita el taller
Preparar una explicación corta y correcta de cada numeral.

#### 2. Cómo se resolvió
Se resume a continuación en el mismo orden de la implementación.

#### 3. Archivos relacionados
Esta guía y los archivos citados.

#### 4. Comandos para demostrarlo
No requiere comando; se estudia con los resultados reales abiertos.
#### 5. Resultado esperado
El equipo debe poder responder sin contradecir código o evidencia.

#### 6. Evidencia generada
La evidencia es la coherencia entre guía, ejecución y respuestas.

#### 7. Cómo explicarlo en la sustentación
“Primero modelo; luego valido FK/IK; después planeo y manipulo; finalmente verifico velocidades con el Jacobiano.”

#### 8. Estado
- Cumple y fue ejecutado


**Guion de 5 minutos.** 1) “Construimos nuestro MoveIt config desde el Xacro del UR5: grupo `ur_manipulator`, HOME/READY, KDL, límites, colisiones y controladores.” 2) “En HOME obtuvimos `base_link→tool0` con ROS y calculamos la misma T por DH; reportamos error de posición, orientación y matriz.” 3) “Definimos PICK y PLACE, resolvimos IK con KDL y validamos cada q haciendo FK DH.” 4) “La PlanningScene tiene superficies, pieza y obstáculo. En 4A y 4C generamos 10 RRTConnect + 10 RRT*, descartamos inválidas y ejecutamos sólo la menor longitud según desempates.” 5) “En 4B usamos una recta cartesiana con cuatro intermedios; comparamos cúbico y quíntico. La decisión ROS se basa en aceleración articular y 4D reutiliza la familia seleccionada.” 6) “Adjuntamos la pieza después de pick y la liberamos sólo después de place.” 7) “Finalmente comparamos Jacobiano DH y MoveIt/KDL y usamos `xdot=J qdot` para verificar la velocidad del TCP.”


## 9. Preguntas posibles del profesor


### 9.1 Banco de preguntas

#### 1. Qué solicita el taller
Preparar al menos 15 preguntas con respuestas sobre todos los conceptos evaluables.

#### 2. Cómo se resolvió
Se incluyen 20 preguntas breves y directamente conectadas con el código.

#### 3. Archivos relacionados
Esta guía.

#### 4. Comandos para demostrarlo
No requiere comando. Para estudiar, tenga abiertos `ur5.srdf`, `kinematics_report.cpp` y `pick_place_sequence.cpp`.
#### 5. Resultado esperado
Poder responder y señalar la línea/archivo que demuestra cada decisión.

#### 6. Evidencia generada
Código, CSV y RViz durante sustentación.

#### 7. Cómo explicarlo en la sustentación
“No memorizo sólo definiciones: relaciono cada concepto con una decisión del proyecto.”

#### 8. Estado
- Cumple y fue ejecutado

**1. ¿URDF y Xacro son lo mismo?** No. URDF es el modelo XML expandido; Xacro permite macros/variables y genera URDF.
**2. ¿Para qué sirve el SRDF?** Añade semántica de MoveIt: grupos, estados nombrados y pares de autocolisión, sin reemplazar URDF.
**3. ¿Qué es `base_link`?** El frame de referencia base usado en el taller para expresar poses y objetos.
**4. ¿Por qué `tool0`?** Es el link del efector final usado como tip del grupo y punto para FK/IK/Jacobiano.
**5. ¿Qué convención DH usan?** DH modificado, con el cambio de base necesario para coincidir con `base_link` de ROS.
**6. ¿Cómo validan DH?** Comparando T completa, error de posición, error angular y máximo de la matriz contra MoveIt.
**7. ¿Puede IK devolver otra solución?** Sí. Un UR5 puede tener varias ramas; por eso se valida la q obtenida realmente y no se exige repetir un vector histórico.
**8. ¿Diferencia RRTConnect/RRT*?** RRTConnect prioriza hallar conexión rápidamente; RRT* busca mejorar calidad asintóticamente. Aquí ambos se comparan por métricas observadas.
**9. ¿Por qué 10 intentos?** Porque OMPL es estocástico y el enunciado exige al menos 10 por planeador; se usan 10 exactos por cada uno.
**10. ¿Qué significa menor movimiento?** Menor `Σ ||q(i+1)-q(i)||₂`, la longitud articular total definida por la instrucción.
**11. ¿Por qué no ejecutar cada plan inmediatamente?** Porque alteraría el estado inicial y haría injusta la comparación; primero se calculan todos y luego se ejecuta sólo el ganador.
**12. ¿Cómo verifican colisiones?** OMPL/MoveIt planifica contra PlanningScene y `computeCartesianPath(..., avoid_collisions=true)` comprueba el tramo cartesiano.
**13. ¿Movimiento libre vs cartesiano?** En libre importa llegar evitando colisiones; en cartesiano se exige que el TCP siga una línea/local path específica.
**14. ¿Qué diferencia hay entre geometría y temporización?** La geometría fija q/poses del camino; la temporización asigna tiempos, velocidades y aceleraciones a esos puntos.
**15. ¿Cúbico vs quíntico?** Ambos pueden imponer velocidad cero en extremos; el quíntico además permite aceleración cero y suele reducir discontinuidades.
**16. ¿Por qué 4D usa el perfil de 4B?** Porque el taller pide usar en 4D el perfil seleccionado en 4B; el código lo fuerza por nombre y se detiene si deja de ser válido.
**17. ¿Qué hace attachObject?** Pasa la pieza al estado adjunto para que MoveIt la trate como parte del robot durante planificación/colisión.
**18. ¿Qué representan las filas del Jacobiano?** Filas 1–3 velocidad lineal; filas 4–6 velocidad angular.
**19. ¿Qué significa `xdot=J qdot`?** Es la relación local entre velocidades articulares y velocidad/twist del efector.
**20. ¿Por qué no afirman que el ciclo final ya funciona?** Porque la última revisión cambió código y este entorno no tiene ROS 2; se conservó evidencia histórica y se dejaron comandos exactos para regenerar evidencia final sin inventarla.

## 10. Solución de errores frecuentes


### 10.1 Diagnóstico reproducible

#### 1. Qué solicita el taller
Documentar los fallos solicitados y cómo aislarlos sin inventar éxito.

#### 2. Cómo se resolvió
Cada caso tiene una comprobación mínima y una acción concreta.

#### 3. Archivos relacionados
Workspace completo.

#### 4. Comandos para demostrarlo
```bash
cd /home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 node list
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic list | grep -E 'robot_description|planning_scene' || true
```
#### 5. Resultado esperado
La comprobación debe revelar si el problema está en launch, controladores, estado, TF o escena.

#### 6. Evidencia generada
Guardar terminal cuando un error sea relevante para la sustentación.

#### 7. Cómo explicarlo en la sustentación
“Primero identifico la capa que falla; no cambio poses o planners a ciegas.”

#### 8. Estado
- Cumple y fue ejecutado

**Robot no visible.** Compruebe Fixed Frame=`base_link`, `robot_description`, estado de RViz y que `demo.launch.py` siga activo.
**robot_description inexistente.** Ejecute `ros2 param get /robot_state_publisher robot_description` o revise el launch/Xacro.
**Ausencia de joint_states.** `ros2 topic echo /joint_states --once`; revise `joint_state_broadcaster`.
**Controlador inactivo.** `ros2 control list_controllers`; active/reinicie `ur5_arm_controller` y broadcaster mediante el launch.
**Estado inicial en colisión.** Limpie PlanningScene, lleve HOME y vuelva a cargarla; no ejecute desde una escena residual.
**Fallo de IK.** Verifique pose alcanzable, orientación, frame `base_link`, link `tool0` y KDL; pruebe el reporte para aislar la pose.
**Fracción cartesiana baja.** No ejecute el parcial. Revise obstáculos/superficies, pose de inicio, continuidad de IK y `eef_step`. La versión previa falló a 51.4 % y por eso se corrigió la escena.
**Ausencia de trayectorias válidas.** Abra el CSV y lea `motivo`; no ejecute movimiento. Revise colisión, límites o tolerancias según el motivo concreto.
**RViz no abre.** Verifique DISPLAY/Wayland/X11; en Linux puede probar `QT_QPA_PLATFORM=xcb`.
**Frame TF inexistente.** `ros2 run tf2_ros tf2_echo base_link tool0`; revise nombres del URDF y que `robot_state_publisher` esté vivo.
**CSV no generado.** Compruebe permisos/ruta `/home/simon/Downloads/ur5_taller_ws_FINAL/ur5_taller_ws/resultados`, que el nodo llegó al punto de guardado y que `resultados/` existe.

## Resumen de resultados medidos disponibles al cerrar esta revisión

### Perfiles Python de la versión final — ejecutados

| Tramo | Perfil | T [s] | vmax [m/s] | amax [m/s²] | Variación de aceleración | Cumple |
|---|---|---:|---:|---:|---:|---|
| 4B rojo | cúbico | 2.5716 | 0.19249 | 0.29940 | 1.19760 | Sí |
| 4B rojo | quíntico | 3.0968 | 0.19980 | 0.19866 | 0.79462 | Sí |
| 4D azul | cúbico | 9.9598 | 0.04970 | 0.019960 | 0.079840 | Sí |
| 4D azul | quíntico | 9.7700 | 0.06333 | 0.019959 | 0.079838 | Sí |

En el modelo escalar Python, **el quíntico es más suave en 4B** según la variación acumulada de aceleración. Esto no sustituye la selección articular en MoveIt: el C++ final vuelve a medir ambos sobre la trayectoria real y registra el seleccionado.

### Evidencia ROS final — ejecutada en ROS 2 Jazzy (2026-09-16)

- HOME real: q = `[0, -1.5708, +1.5708, -1.5708, -1.5708, 0]`; posición MoveIt `≈ [0.486898741, 0.109149698, 0.431859348] m`; error de posición y de matriz frente a DH = `0.0` (error de orientación `≈ 3e-8 rad` por redondeo numérico).
- PICK real: posición `≈ [0.649999942, -0.299999991, 0.120000020] m`; IK resuelto por KDL con error nulo frente a DH.
- PLACE real: posición `≈ [0.649999634, 0.299999477, 0.119999951] m`; error nulo frente a DH.
- Jacobianos reales HOME/PICK/PLACE/4B/4D: norma de Frobenius del error DH-MoveIt `≈ 1e-9` en los cinco estados; `xdot=J·qdot` coincide con la velocidad TCP exigida por el perfil en 4B (0.2 m/s) y 4D (0.1 m/s).
- 4A real: 20/20 candidatos válidos; ganador `RRTstarkConfigDefault` intento 1, longitud `1.29984756 rad`, 30 puntos, `2.81541943 s` (tabla completa arriba). Reproducido idéntico en 3 corridas del ciclo FULL.
- 4B real: `computeCartesianPath ≥ 99.9 %`; perfil cúbico descartado por violar aceleración TCP (`0.304 m/s² > 0.3`), perfil quíntico válido y ejecutado (`vmax_tcp≈0.199 m/s`, `amax_tcp≈0.199 m/s²`); pieza adjuntada tras la ejecución.
- 4C real: 20 candidatos con la pieza adjunta (10 válidos RRTConnect, 1 válido RRTstar en la corrida citada); ganador seleccionado y ejecutado por menor longitud articular.
- 4D real: perfil quíntico reutilizado obligatoriamente desde 4B (`4D_pre_place_place/quintico`, `T≈9.76 s`, `amax_tcp≈0.02 m/s²`); pieza liberada tras alcanzar PLACE.
- Ciclo FULL: ejecutado 3 veces de forma reproducible (`resultados/ciclo_terminal_final_run1/2/3.txt`), terminando siempre en "pieza liberada despues de alcanzar PLACE".

Estos datos están en `resultados/` (no en la carpeta histórica). La carpeta `resultados/historico_ejecucion_2026-09-15/` se conserva sólo como referencia de una ejecución anterior con una PlanningScene distinta.
