# Comandos rápidos de sustentación

## Modelo UR5 y MoveIt

```bash
./taller.sh modelo
```

Expande el Xacro, corre `check_urdf` y muestra el grupo de planeación, los estados nombrados, el solver KDL y la configuración OMPL/controladores.

## HOME: DH vs ROS

```bash
./taller.sh home
```

Lleva el robot a HOME y compara la transformación `base_link -> tool0` calculada por MoveIt/TF contra el modelo DH.

## IK PICK / PLACE

```bash
./taller.sh ik
```

Resuelve la IK de PICK y PLACE con MoveIt/KDL y verifica esas articulaciones contra el modelo DH.

## PlanningScene

```bash
./taller.sh escena
```

Aplica la escena (pick_surface, place_surface, workpiece, obstacle) y confirma los 4 objetos.

## RRTConnect vs RRTstar

```bash
./taller.sh 4a
```

HOME -> PRE-PICK: 20 candidatos (10 por planeador), tabla comparativa y ejecución del ganador.

## Cúbico vs quíntico

```bash
./taller.sh 4b
```

PRE-PICK -> PICK con `computeCartesianPath`, comparación de perfiles y adjuntado de la pieza.

## PICK -> PRE-PLACE

```bash
./taller.sh 4c
```

Confirma la pieza adjunta y repite la comparación de planeadores con esa geometría extra.

## PLACE

```bash
./taller.sh 4d
```

Reutiliza el perfil de 4B, verifica límites de velocidad/aceleración y libera la pieza.

## Jacobiano

```bash
./taller.sh jacobiano
```

J_DH vs J_MoveIt/KDL en HOME, PICK, PLACE, 4B y 4D, con verificación de `xdot = J·qdot`.

## Ciclo completo

```bash
./taller.sh ciclo
```

HOME -> PRE-PICK -> PICK -> ATTACH -> PRE-PLACE -> PLACE -> DETACH, con progreso en vivo.

## MATLAB

```bash
./taller.sh matlab
```

Corre la validación DH y genera las gráficas de evidencia en MATLAB/Octave (si está instalado).

## Verificar todo

```bash
./taller.sh verificar
```

Auditoría estática completa del repositorio (estructura, imágenes, referencias, IA).

## Ejecutar todas las pruebas

```bash
./taller.sh todo
```

Corre la secuencia completa del taller en orden seguro.
