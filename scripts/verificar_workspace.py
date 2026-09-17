#!/usr/bin/env python3
"""Verificación estática reproducible del workspace del taller UR5.

No reemplaza colcon/check_urdf ni una ejecución de MoveIt. Sirve para detectar
archivos, estructura documental y configuraciones faltantes antes de compilar.
"""
from __future__ import annotations

import ast
import csv
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None

ROOT = Path(__file__).resolve().parents[1]
OK = []
FAIL = []


def check(condition: bool, label: str) -> None:
    (OK if condition else FAIL).append(label)


def text(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


required_files = [
    "README.md",
    "GUIA_COMPLETA_TALLER.md",
    "EVIDENCIA_COMPLETA_TALLER.md",
    "COMANDOS_RAPIDOS.md",
    "LIMPIEZA_ENTREGA.md",
    "taller.sh",
    "src/ur5_description/urdf/ur5.urdf.xacro",
    "src/ur5_moveit_config/config/ur5.srdf",
    "src/ur5_moveit_config/config/kinematics.yaml",
    "src/ur5_moveit_config/config/joint_limits.yaml",
    "src/ur5_moveit_config/config/ompl_planning.yaml",
    "src/ur5_moveit_config/config/ros2_controllers.yaml",
    "src/ur5_pick_place/src/planning_scene_setup.cpp",
    "src/ur5_pick_place/src/pick_place_sequence.cpp",
    "src/ur5_pick_place/src/kinematics_report.cpp",
    "src/ur5_pick_place/scripts/trajectory_profiles.py",
    "matlab/UR5_DH_VALIDACION.m",
    "resultados/tablas/resumen_perfiles.csv",
    "kuka_kr6_matlab/Matlab_TallerIK.m",
    "kuka_kr6_matlab/README.md",
]
for rel in required_files:
    p = ROOT / rel
    check(p.exists() and p.stat().st_size > 0, f"archivo presente y no vacío: {rel}")

# XML/Xacro structural parsing. This is not xacro expansion.
xml_files = list((ROOT / "src").rglob("package.xml")) + [ROOT / "src/ur5_moveit_config/config/ur5.srdf"]
xml_files += list((ROOT / "src/ur5_description/urdf").rglob("*.xacro"))
for p in xml_files:
    try:
        ET.parse(p)
        check(True, f"XML bien formado: {p.relative_to(ROOT)}")
    except Exception as exc:
        check(False, f"XML bien formado: {p.relative_to(ROOT)} ({exc})")

# YAML syntax.
if yaml is None:
    check(False, "PyYAML disponible para revisar YAML")
else:
    check(True, "PyYAML disponible para revisar YAML")
    for p in (ROOT / "src/ur5_moveit_config/config").glob("*.yaml"):
        try:
            yaml.safe_load(p.read_text(encoding="utf-8"))
            check(True, f"YAML válido: {p.relative_to(ROOT)}")
        except Exception as exc:
            check(False, f"YAML válido: {p.relative_to(ROOT)} ({exc})")

# Python syntax.
py = ROOT / "src/ur5_pick_place/scripts/trajectory_profiles.py"
try:
    ast.parse(py.read_text(encoding="utf-8"))
    check(True, "trajectory_profiles.py compila sintácticamente")
except SyntaxError as exc:
    check(False, f"trajectory_profiles.py compila sintácticamente ({exc})")

srdf = text("src/ur5_moveit_config/config/ur5.srdf")
check('group name="ur_manipulator"' in srdf, "SRDF contiene ur_manipulator")
check('group_state name="HOME"' in srdf, "SRDF contiene HOME")
check('group_state name="READY"' in srdf, "SRDF contiene READY")
check(srdf.count("disable_collisions") >= 6, "SRDF contiene matriz de autocolisiones")

kin = text("src/ur5_moveit_config/config/kinematics.yaml")
check("KDLKinematicsPlugin" in kin, "KDL configurado")

ompl = text("src/ur5_moveit_config/config/ompl_planning.yaml")
check("RRTConnectkConfigDefault" in ompl, "OMPL contiene RRTConnect")
check("RRTstarkConfigDefault" in ompl, "OMPL contiene RRTstar")

scene = text("src/ur5_pick_place/src/planning_scene_setup.cpp")
for object_id in ("pick_surface", "place_surface", "workpiece", "obstacle"):
    check(object_id in scene, f"PlanningScene contiene {object_id}")

seq = text("src/ur5_pick_place/src/pick_place_sequence.cpp")
checks = {
    "INTENTOS=10": bool(re.search(r"constexpr\s+int\s+INTENTOS\s*=\s*10", seq)),
    "computeCartesianPath": "computeCartesianPath" in seq,
    "4 waypoints intermedios": "for (int i = 1; i <= 5; ++i)" in seq,
    "RRTConnect": "RRTConnectkConfigDefault" in seq,
    "RRTstar": "RRTstarkConfigDefault" in seq,
    "CSV candidatos": "guardar_candidatos" in seq,
    "selección antes de execute": seq.find("guardar_candidatos") < seq.find("grupo.execute(mejor->plan)"),
    "attach después de 4B": "grupo.attachObject" in seq,
    "detach después de 4D": "grupo.detachObject" in seq,
    "perfil 4D forzado desde 4B": "perfil_4b, nullptr" in seq,
    "evidencia perfil seleccionado": "guardar_perfil_seleccionado" in seq,
    "medición TCP con J*qdot": "jacobiano * qdot" in seq and "max_vel_tcp" in seq,
    "límites TCP en C++": "viola velocidad TCP" in seq and "viola aceleracion TCP" in seq,
    "detención sin candidato": "no hay candidato valido; no se mueve el robot" in seq,
}
for label, condition in checks.items():
    check(condition, f"secuencia: {label}")

kin_cpp = text("src/ur5_pick_place/src/kinematics_report.cpp")
for token in ("HOME", "PICK", "PLACE", '"4B"', '"4D"', "CURRENT"):
    check(token in kin_cpp, f"kinematics_report soporta {token}")
for token in ("Cuaternion MoveIt", "Error maximo de la matriz homogenea", "getJacobian", "xdot exigida por perfil"):
    check(token in kin_cpp, f"kinematics_report reporta: {token}")

# Guide mandatory headings from the explicit instructions.
guide = text("GUIA_COMPLETA_TALLER.md")
mandatory_headings = [
    "# Guía completa del taller UR5 con ROS 2 y MoveIt 2",
    "## 0. Preparación del workspace",
    "## 1. Modelo del manipulador",
    "### 1.1 Paquete ur5_description", "### 1.2 Validación URDF/Xacro",
    "### 1.3 Geometrías visuales y de colisión", "### 1.4 Grupo de planeación",
    "### 1.5 Estados HOME y READY", "### 1.6 Límites articulares",
    "### 1.7 Matriz de autocolisiones", "### 1.8 Solver KDL",
    "### 1.9 Visualización en RViz", "### 1.10 MoveIt Setup Assistant",
    "## 2. Transformación homogénea en HOME",
    "### 2.1 Llevar el robot a HOME", "### 2.2 Consultar base_link -> tool0",
    "### 2.3 Frames de cada articulación", "### 2.4 Árbol TF",
    "### 2.5 Pose desde MoveIt", "### 2.6 Transformación DH",
    "### 2.7 Comparación MoveIt/TF2 contra DH", "### 2.8 Errores de posición y orientación",
    "## 3. Cinemática inversa de pick y place",
    "### 3.1 Definición de pick", "### 3.2 IK de pick", "### 3.3 Valores articulares de pick",
    "### 3.4 Verificación DH de pick", "### 3.5 Definición de place", "### 3.6 IK de place",
    "### 3.7 Valores articulares de place", "### 3.8 Verificación DH de place",
    "### 3.9 Error MoveIt frente a DH",
    "## 4. Ciclo de pick-and-place",
    "### 4.0 PlanningScene", "### 4.0.1 Mesa", "### 4.0.2 Pieza", "### 4.0.3 Obstáculo",
    "### 4A. HOME -> pre-pick", "### 4A.1 RRTConnect", "### 4A.2 RRTstar",
    "### 4A.3 Múltiples intentos", "### 4A.4 Métricas", "### 4A.5 Descarte de trayectorias",
    "### 4A.6 Selección de la mejor", "### 4B. Pre-pick -> pick", "### 4B.1 Waypoints",
    "### 4B.2 Perfil cúbico", "### 4B.3 Perfil quíntico", "### 4B.4 Velocidad y aceleración",
    "### 4B.5 Selección del perfil", "### 4C. Pick -> pre-place", "### 4C.1 Adjuntar la pieza",
    "### 4C.2 Generar candidatos", "### 4C.3 Evaluar candidatos", "### 4C.4 Ejecutar la mejor trayectoria",
    "### 4D. Pre-place -> place", "### 4D.1 Aproximación cartesiana", "### 4D.2 Verificar el perfil",
    "### 4D.3 Liberar la pieza", "### 4E. Ciclo completo en RViz",
    "## 5. Jacobiano", "### 5.1 Jacobiano analítico", "### 5.2 Jacobiano MoveIt/KDL",
    "### 5.3 Comparación numérica", "### 5.4 Error máximo", "### 5.5 Relación x_dot = J q_dot",
    "### 5.6 Verificación en 4B", "### 5.7 Verificación en 4D",
    "### 5.8 Comandos para HOME, pick, place y estado actual",
    "## 6. Resultados y evidencias", "## 7. Ejecución completa desde cero",
    "## 8. Guion para la sustentación", "## 9. Preguntas posibles del profesor",
    "## 10. Solución de errores frecuentes",
]
for heading in mandatory_headings:
    check(heading in guide, f"guía contiene: {heading}")

# Every numbered subsection must contain the 8 mandated fields before the next numbered subsection.
numbered_positions = list(re.finditer(r"^###\s+(?:\d+(?:\.\d+)*|4[A-E](?:\.\d+)?)\.?\s+.+$", guide, flags=re.M))
required_fields = [
    "#### 1. Qué solicita el taller", "#### 2. Cómo se resolvió", "#### 3. Archivos relacionados",
    "#### 4. Comandos para demostrarlo", "#### 5. Resultado esperado", "#### 6. Evidencia generada",
    "#### 7. Cómo explicarlo en la sustentación", "#### 8. Estado",
]
for i, match in enumerate(numbered_positions):
    end = numbered_positions[i + 1].start() if i + 1 < len(numbered_positions) else len(guide)
    section = guide[match.start():end]
    # Skip the standalone historical table if its title ever matches (it currently does not).
    for field in required_fields:
        check(field in section, f"plantilla completa: {match.group(0)} -> {field[5:]}")

allowed_states = {
    "Cumple y fue ejecutado.", "Cumple por revisión estática.", "Cumple parcialmente.",
    "No se pudo verificar.", "Pendiente.",
    # Guide bullets omit final period visually; accepted below as normalized form.
}
state_lines = re.findall(r"#### 8\. Estado\s*\n- ([^\n]+)", guide)
check(len(state_lines) >= len(numbered_positions), "cada subsección tiene estado")
for state in state_lines:
    normalized = state if state.endswith('.') else state + '.'
    check(normalized in allowed_states, f"estado permitido: {state}")

check(guide.count("**1.") >= 1 and len(re.findall(r"^\*\*\d+\.", guide, flags=re.M)) >= 15,
      "guía contiene al menos 15 preguntas")

# Final Python profile outputs are non-empty and all rows comply.
summary = ROOT / "resultados/tablas/resumen_perfiles.csv"
if summary.exists():
    rows = list(csv.DictReader(summary.open(encoding="utf-8")))
    check(len(rows) == 4, "resumen de perfiles contiene 4 filas")
    check(all(r.get("cumple_limites") == "True" for r in rows), "los 4 perfiles Python cumplen límites")

# Historical evidence is clearly separated.
hist = ROOT / "resultados/historico/ejecucion_2026-09-15"
check((hist / "LEEME.txt").exists(), "historial ROS tiene aviso de trazabilidad")
check((hist / "ciclo_terminal.txt").exists(), "historial conserva terminal real")
check((hist / "trayectorias_4A_home_pre_pick.csv").exists(), "historial conserva candidatos 4A reales")

# KUKA KR-6 (entrega independiente en MATLAB).
kuka = ROOT / "kuka_kr6_matlab/Matlab_TallerIK.m"
if kuka.exists():
    kuka_txt = kuka.read_text(encoding="utf-8")
    for token, label in [
        ("num_configuraciones = 8", "KUKA: 8 configuraciones por punto"),
        ("SlnSet = zeros(num_puntos, 6, num_configuraciones)", "KUKA: SlnSet n x 6 x 8"),
        ("SlnSetVer", "KUKA: filtrado SlnSetVer"),
        ("isreal(capa_actual)", "KUKA: filtrado de valores imaginarios"),
        ("~isnan(capa_actual)", "KUKA: filtrado de NaN"),
        ("calcular_jacobiano_completo", "KUKA: cálculo de Jacobiano"),
        ("calcular_FK_completa", "KUKA: cinemática directa"),
        ("dibujar_frame", "KUKA: animación/visualización de frames"),
    ]:
        check(token in kuka_txt, label)
else:
    check(False, "kuka_kr6_matlab/Matlab_TallerIK.m presente")

# taller.sh es ejecutable y define todos los subcomandos pedidos.
taller = ROOT / "taller.sh"
if taller.exists():
    check(os.access(taller, os.X_OK), "taller.sh es ejecutable")
    taller_txt = taller.read_text(encoding="utf-8")
    for sub in ["ayuda", "preparar", "compilar", "modelo", "home", "ik", "escena",
                "4a", "4b", "4c", "4d", "jacobiano", "ciclo", "matlab", "verificar", "todo"]:
        check(re.search(rf"(?:^|\s|\|){re.escape(sub)}(?:\||\))", taller_txt, re.M) is not None,
              f"taller.sh define el comando: {sub}")

print("VERIFICACION ESTATICA DEL WORKSPACE UR5")
print(f"OK: {len(OK)}")
print(f"FALLOS: {len(FAIL)}")
for label in OK:
    print(f"[OK] {label}")
for label in FAIL:
    print(f"[FALLO] {label}")

print("\nNOTA: esta prueba no sustituye xacro/check_urdf, colcon build, ROS 2, MoveIt ni RViz.")
sys.exit(1 if FAIL else 0)
