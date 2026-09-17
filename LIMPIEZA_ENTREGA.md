# Limpieza y reorganización de la entrega

Este documento explica qué se hizo con cada archivo antes de dejar el repositorio en su
forma final. Se escribió antes de mover o borrar nada.

## Archivos que se conservan (sin mover)

| Archivo | Motivo |
|---|---|
| `src/ur5_description/` | Paquete real del modelo UR5 (URDF/Xacro, mallas, config). |
| `src/ur5_moveit_config/` | Configuración MoveIt2 real (SRDF, kinematics, OMPL, controladores, RViz). |
| `src/ur5_pick_place/` | Código C++/Python del ciclo pick-and-place, escena y reportes cinemáticos. |
| `matlab/UR5_DH_HOME.m` | Validación DH del UR5 en HOME, comparación contra `loadrobot`. |
| `matlab/UR5_DH_VALIDACION.m` | Validación DH de HOME/PICK/PLACE sin toolbox. |
| `scripts/verificar_workspace.py` | Auditoría estática reproducible del workspace. |
| `scripts/verificar_ros_final.sh` | Script de referencia de compilación/ejecución. |
| `Matlab_TallerIK` | Entrega independiente del KUKA KR-6. Se mueve de carpeta (ver abajo) pero no se borra ni se mezcla con el UR5. |
| `.gitignore` | Ya excluye `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`. |

## Archivos que se reorganizan

| Origen | Destino | Motivo |
|---|---|---|
| `Matlab_TallerIK` | `kuka_kr6_matlab/Matlab_TallerIK.m` | Es un script MATLAB sin extensión `.m`; se renombra para que MATLAB/Octave lo reconozca y se agrupa con su propia entrega, separada del UR5. |
| `resultados/historico_ejecucion_2026-09-15/` | `resultados/historico/ejecucion_2026-09-15/` | Evidencia real pero de una versión anterior del código (antes de las correcciones de esta sesión). Se conserva como histórico, no como resultado final. |
| `resultados/check_urdf.txt`, `compilacion_final.txt` (log), `kinematics_*.{txt,csv}`, `tf_*.txt`, `trayectorias_4A_home_pre_pick.csv`, `trayectorias_4C_pick_pre_place.csv`, `verificacion_estatica.txt` | `resultados/ros2/` | Evidencia estructurada generada directamente por los ejecutables ROS2/C++. |
| `resultados/terminal_4A.txt` ... `terminal_4D.txt`, `ciclo_terminal_final.txt`, `move_home_terminal.txt`, `perfiles_terminal.txt` | `resultados/logs/` | Volcados de terminal (`tee`) de cada ejecución. |
| `resultados/ciclo_terminal_final_run1/2/3.txt` | `resultados/logs/reproducibilidad/` | Las 3 corridas que demuestran reproducibilidad del ciclo FULL (pedido explícito: conservarlas). |
| `resultados/resumen_perfiles.csv`, `perfil_4B_rojo_*.csv`, `perfil_4D_azul_*.csv`, `waypoints_4B_*.csv`, `waypoints_4D_*.csv`, `perfiles_4B_pre_pick_pick.csv`, `perfiles_4D_pre_place_place.csv`, `perfil_seleccionado_4B_pre_pick_pick.txt`, `perfil_seleccionado_4D_pre_place_place.txt` | `resultados/tablas/` | Tablas numéricas de comparación de perfiles (modelo escalar Python y trayectoria articular real). |
| `resultados/rviz_captura.png`, `comparacion_perfiles_4B_rojo.png`, `comparacion_perfiles_4D_azul.png`, `frames_2026-09-16_08.45.33.pdf`, `frames_2026-09-16_08.45.33.gv` | `resultados/imagenes/` | Gráficas y capturas. |
| `README.md`, `GUIA_COMPLETA_TALLER.md` | (contenido actualizado in situ) | Se actualizan las rutas de evidencia citadas para que apunten a la nueva estructura. |

## Archivos que se eliminan

| Archivo | Motivo |
|---|---|
| `resultados/ciclo_terminal_checklist.txt` | Corrida exploratoria hecha para probar el checklist en pantalla durante el desarrollo; el contenido (mismo ciclo FULL) ya está cubierto por `run1/2/3` y por el `ciclo_terminal_final.txt` que genera `taller.sh ciclo`. |
| `resultados/ciclo_terminal_visible.txt`, `ciclo_terminal_visible2.txt`, `ciclo_terminal_visible3.txt` | Corridas exploratorias hechas para verificar visualmente el fantasma de RViz durante el desarrollo. No aportan datos distintos a `run1/2/3`. |
| `resultados/ciclo_terminal_home_full.txt` | Corrida exploratoria repetida tras agregar el checklist; redundante con `run1/2/3`. |
| `resultados/terminal_pick_place_directo.txt`, `resultados/trayectorias_pick_a_place_directo.csv` | Corresponden a un modo de demostración rápida (`solo_pick_place`) que no forma parte de los 9 numerales del taller ni de los comandos pedidos en `taller.sh`. Se documenta su existencia en el código pero no se presenta como evidencia formal. |
| `ENTREGA_FINAL.md` | Su contenido (qué se corrigió, tabla de estado) se fusiona dentro de `EVIDENCIA_COMPLETA_TALLER.md` para no duplicar documentación. |
| `REVISION_TALLER.md` | Igual que el anterior: su historial de problemas encontrados/corregidos se resume dentro de `EVIDENCIA_COMPLETA_TALLER.md`. |
| `scripts/__pycache__/`, `src/ur5_pick_place/scripts/__pycache__/` | Caché de Python regenerable, ya cubierta por `.gitignore` (no estaba rastreada por git). |
| `resultados/.gitkeep` | Ya no hace falta: la carpeta `resultados/` tendrá contenido real versionado. |

Antes de borrar cada uno de estos archivos se comparó su contenido contra `run1/2/3` y contra
los resultados finales para confirmar que no contienen ninguna corrida o dato que no esté ya
cubierto en otro lugar.
