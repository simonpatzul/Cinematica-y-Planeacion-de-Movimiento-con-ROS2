# Taller de Cinemática y Planeación de Movimiento

Este repositorio contiene dos entregas independientes, cada una con su propio robot y su propia documentación:

- **Taller IK** — KUKA KR-6, cinemática inversa analítica en MATLAB.
- **Taller UR5** — ROS 2 Jazzy + MoveIt 2.

No se mezclan resultados entre las dos entregas.

## Taller IK — KUKA KR-6 (MATLAB)

Cinemática inversa analítica exacta (desacople cinemático, hasta 8 configuraciones por punto), filtrado por capas, Jacobiano y animación 3D con frames `{0}` y `{6}`.

Video explicativo: https://www.youtube.com/watch?v=DNgPkYz-wLw

Ver [kuka_kr6_matlab/README.md](kuka_kr6_matlab/README.md) para el detalle completo y cómo ejecutarlo.

## Taller UR5 — ROS 2 Jazzy + MoveIt 2

Configuración propia de MoveIt 2 para un UR5 clásico construida desde su URDF/Xacro: PlanningScene, comparación RRTConnect/RRTstar, ciclo pick-and-place con perfiles cúbico/quíntico, validación DH y Jacobiano.

### Requisitos

- Ubuntu con ROS 2 Jazzy, MoveIt 2, RViz 2, ros2_control, `xacro`, `check_urdf`.
- Python 3 con NumPy y Matplotlib.
- MATLAB u Octave (opcional, sin Robotics System Toolbox).

### Estructura

Ver el árbol completo y la explicación de cada carpeta en [EVIDENCIA_COMPLETA_TALLER.md](EVIDENCIA_COMPLETA_TALLER.md#2-estructura-del-proyecto).

### Compilación

```bash
cd <ruta_del_repositorio>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Ejecución rápida

```bash
./taller.sh ayuda
```

Para correr el taller completo, un comando por numeral:

```bash
./taller.sh todo
```

Cada comando (`modelo`, `home`, `ik`, `escena`, `4a`, `4b`, `4c`, `4d`, `jacobiano`, `ciclo`, `matlab`, `verificar`) ejecuta solo esa prueba, muestra resultados numéricos reales y dice dónde queda la evidencia. Ver [COMANDOS_RAPIDOS.md](COMANDOS_RAPIDOS.md) para la lista completa.

### Documentación

- [EVIDENCIA_COMPLETA_TALLER.md](EVIDENCIA_COMPLETA_TALLER.md) — evidencia principal, numeral por numeral.
- [COMANDOS_RAPIDOS.md](COMANDOS_RAPIDOS.md) — comandos para la sustentación.
- [GUIA_COMPLETA_TALLER.md](GUIA_COMPLETA_TALLER.md) — guía extendida con el detalle de cada comando ROS2.

Los resultados se guardan en `resultados/` (separados en `ros2/`, `logs/`, `tablas/`, `imagenes/`, `matlab/` e `historico/`). Ver [LIMPIEZA_ENTREGA.md](LIMPIEZA_ENTREGA.md) para el detalle de la organización.
