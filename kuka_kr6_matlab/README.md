# Taller de Cinemática — KUKA KR-6 (MATLAB)

Entrega independiente del taller de cinemática y trayectoria del robot KUKA KR-6 (6R). No tiene relación con el workspace ROS2/MoveIt del UR5 de la carpeta raíz; se conserva aparte a propósito.

## Objetivo

Generar una trayectoria cartesiana de 25 puntos, resolver la cinemática inversa analítica exacta (desacople cinemático) para hasta 8 configuraciones por punto, filtrar las configuraciones válidas por capas completas, calcular el Jacobiano y animar el recorrido con los frames `{0}` y `{6}`.

## Robot

KUKA KR-6, 6 grados de libertad rotacionales. Parámetros DH modificados (en mm):

| Parámetro | Valor |
|---|---:|
| d1 | 675 |
| a2 | 260 |
| a3 | 680 |
| a4 | -35 |
| d4 | 670 |
| d6 | 115 |

## Trayectoria

25 puntos cartesianos: 3 "niveles" de un patrón tipo árbol (cada uno con 6 puntos formando un lazo alrededor de un centro común) más 6 puntos de un "tronco" central que desciende en Z. La posición varía en los 25 puntos.

La orientación del efector (`R_deseada`) es fija para los 25 puntos: la herramienta apunta siempre en la misma dirección mientras la posición recorre los 25 puntos del árbol y el tronco.

## Cinemática inversa

Para cada uno de los 25 puntos se resuelve la IK analítica exacta (sin optimización numérica) mediante desacople cinemático: primero el centro de la muñeca `P_wc = P_deseada - d6·R(:,3)`, luego hombro/codo (2 soluciones cada uno) y luego la muñeca esférica (2 soluciones), dando hasta 8 configuraciones por punto.

`SlnSet` es una matriz de `25 x 6 x 8` (puntos × GDL × configuraciones). `Slnsetpoint_1` es el corte `SlnSet(1,:,:)`, reordenado como `6 x 8` (una columna por configuración del punto 1).

## Filtrado (SlnSetVer)

Una configuración (capa `k` de `SlnSet(:,:,k)`) se acepta completa o se descarta completa: se exige que **ninguno** de los 25 puntos de esa capa tenga valores imaginarios o `NaN`. No se mezclan configuraciones de capas distintas para armar una trayectoria híbrida. `SlnSetVer` es la primera capa que pasa el filtro completo, usada como trayectoria definitiva para el Jacobiano y la animación.

## Jacobiano y velocidades articulares

`calcular_jacobiano_completo` arma el Jacobiano geométrico (6×6) a partir de las transformaciones DH acumuladas. Entre cada par de puntos consecutivos de `SlnSetVer` se estima la velocidad cartesiana deseada (posición dividida entre el tiempo entre puntos) y se obtiene `qdot = pinv(J)·V_cartesiana`.

## Animación

La figura dibuja la trayectoria cartesiana deseada (línea punteada), el rastro real del efector obtenido por cinemática directa a partir de `SlnSetVer` (línea roja), y los frames `{0}` (base) y `{6}` (efector) en el primer punto, el punto medio y el último punto del recorrido.

## Cómo ejecutar

Con MATLAB:

```matlab
cd('kuka_kr6_matlab');
Matlab_TallerIK
```

Con Octave (sin Robotics System Toolbox, el script no lo necesita):

```bash
octave --no-gui --eval "cd('kuka_kr6_matlab'); Matlab_TallerIK"
```

## Archivos generados

El script imprime en consola: dimensiones de `SlnSet`, `Slnsetpoint_1`, el resultado del filtrado por capas, la capa asignada a `SlnSetVer` y el primer vector articular en radianes. La figura 3D con la trayectoria y los frames se muestra en pantalla (`figure(1)`); si se ejecuta con `./taller.sh matlab` desde la raíz del repositorio, se guardan capturas de las gráficas del UR5 en `../resultados/matlab/` (este script del KUKA es independiente de ese flujo).
