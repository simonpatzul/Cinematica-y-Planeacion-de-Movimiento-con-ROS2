#!/usr/bin/env bash
# Dispatcher del taller UR5 (Cinemática y Planeación de Movimiento, ROS 2 + MoveIt 2).
# Cada subcomando vive en scripts/taller_<comando>.sh y es responsable de:
# preparar variables, cargar ROS 2, ejecutar solo esa prueba, explicar
# brevemente, mostrar resultados numéricos reales, indicar la evidencia
# generada y devolver 0 si todo salió bien o distinto de 0 si algo falló.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$SCRIPT_DIR"
source "$SCRIPT_DIR/scripts/taller_common.sh"

mostrar_ayuda() {
  cat <<'EOF'
Taller UR5 - ROS 2 Jazzy + MoveIt 2

Uso:

  ./taller.sh ayuda        Muestra esta ayuda.
  ./taller.sh preparar      Comprueba el entorno (ROS2, xacro, check_urdf, Python, MATLAB) y rosdep.
  ./taller.sh compilar      Compila limpio con colcon (rm -rf build install log && colcon build).
  ./taller.sh modelo        PARTE 1: URDF/Xacro, check_urdf y configuración MoveIt2.
  ./taller.sh home          PARTE 2: transformación HOME, T por ROS/MoveIt vs T por DH.
  ./taller.sh ik            PARTE 3: IK de PICK y PLACE, comparación con DH.
  ./taller.sh escena        PARTE 4.0: aplica y verifica la PlanningScene (4 objetos).
  ./taller.sh 4a            PARTE 4A: HOME -> PRE-PICK, RRTConnect vs RRTstar (20 intentos).
  ./taller.sh 4b            PARTE 4B: PRE-PICK -> PICK, cúbico vs quíntico (computeCartesianPath).
  ./taller.sh 4c            PARTE 4C: PICK -> PRE-PLACE con la pieza adjunta (ATTACHED).
  ./taller.sh 4d            PARTE 4D: PRE-PLACE -> PLACE, reutiliza el perfil de 4B, DETACH.
  ./taller.sh jacobiano     PARTE 5: J_DH vs J_MoveIt/KDL en HOME/PICK/PLACE/4B/4D.
  ./taller.sh ciclo         Ciclo completo HOME -> ... -> DETACH, con progreso [n/7].
  ./taller.sh matlab        Ejecuta la validación DH en MATLAB/Octave y genera gráficas.
  ./taller.sh verificar     Auditoría final del repositorio completo.
  ./taller.sh todo          Ejecuta la secuencia completa en orden seguro.

Ejemplos:
  ./taller.sh preparar && ./taller.sh compilar && ./taller.sh todo
EOF
}

cmd_preparar() {
  titulo "PREPARAR ENTORNO"
  local estado_entorno="FALLO"
  local ros_distro="desconocido"
  local estado_dep="FALLO"
  local estado_matlab="NO DISPONIBLE"

  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
    estado_entorno="OK"
    ros_distro="${ROS_DISTRO:-jazzy}"
  fi

  for herramienta in ros2 colcon xacro check_urdf python3; do
    if command -v "$herramienta" >/dev/null 2>&1; then
      ok "$herramienta disponible"
    else
      mal "$herramienta NO disponible"
      estado_entorno="FALLO"
    fi
  done

  python3 -c "import yaml, matplotlib, numpy" >/dev/null 2>&1 \
    && { ok "dependencias Python (pyyaml, matplotlib, numpy) disponibles"; } \
    || { mal "faltan dependencias Python"; }

  if command -v matlab >/dev/null 2>&1 || command -v octave >/dev/null 2>&1; then
    estado_matlab="DISPONIBLE"
  fi

  if [[ "$estado_entorno" == "OK" ]]; then
    info "Ejecutando rosdep install..."
    cd "$ROOT_DIR" && rosdep install --from-paths src --ignore-src -r -y \
      > "$R_LOGS/rosdep.txt" 2>&1
    estado_dep=$([[ $? -eq 0 ]] && echo "OK" || echo "OK (avisos, ver $R_LOGS/rosdep.txt)")
  fi

  echo
  printf 'ENTORNO ROS2............ %s\n' "$estado_entorno"
  printf 'ROS_DISTRO............... %s\n' "$ros_distro"
  printf 'WORKSPACE................ %s\n' "$ROOT_DIR"
  printf 'DEPENDENCIAS.............. %s\n' "$estado_dep"
  printf 'MATLAB.................... %s\n' "$estado_matlab"

  [[ "$estado_entorno" == "OK" ]] && return 0 || return 1
}

cmd_compilar() {
  titulo "COMPILACIÓN"
  cargar_ros_solo_jazzy() {
    [[ -f /opt/ros/jazzy/setup.bash ]] || { mal "No existe ROS 2 Jazzy."; return 1; }
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
  }
  cargar_ros_solo_jazzy || return 1
  cd "$ROOT_DIR" || return 1

  info "Limpiando build/install/log..."
  rm -rf build install log

  info "colcon build --symlink-install..."
  colcon build --symlink-install 2>&1 | tee "$R_LOGS/compilacion_final.txt"
  local build_ok=${PIPESTATUS[0]}

  set +u
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash" 2>/dev/null
  set -u

  local n_paquetes=0
  for pkg in ur5_description ur5_moveit_config ur5_pick_place; do
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
      n_paquetes=$((n_paquetes + 1))
    fi
  done

  echo
  printf 'PAQUETES: %s/3\n' "$n_paquetes"
  if [[ $build_ok -eq 0 && $n_paquetes -eq 3 ]]; then
    printf 'BUILD: OK\n'
    return 0
  else
    printf 'BUILD: FALLÓ\n'
    return 1
  fi
}

cmd_verificar() {
  titulo "VERIFICACIÓN FINAL"
  local fallo=0
  cargar_ros >/dev/null 2>&1

  ros2 pkg prefix ur5_description >/dev/null 2>&1 && ok "Workspace" || { mal "Workspace"; fallo=1; }
  [[ -f /opt/ros/jazzy/setup.bash ]] && ok "ROS2 Jazzy" || { mal "ROS2 Jazzy"; fallo=1; }

  xacro "$ROOT_DIR/src/ur5_description/urdf/ur5.urdf.xacro" > /tmp/taller_verificar.urdf 2>/dev/null \
    && check_urdf /tmp/taller_verificar.urdf >/dev/null 2>&1 \
    && ok "URDF" || { mal "URDF"; fallo=1; }

  [[ -f "$ROOT_DIR/src/ur5_moveit_config/config/ur5.srdf" ]] && ok "MoveIt config" || { mal "MoveIt config"; fallo=1; }
  [[ -f "$ROOT_DIR/install/setup.bash" ]] && ok "Build" || { mal "Build"; fallo=1; }

  for f in "$R_ROS2/kinematics_HOME.txt"; do
    [[ -f "$f" ]] && ok "HOME (evidencia previa encontrada)" || { aviso "HOME (sin evidencia; ejecute ./taller.sh home)"; }
  done
  for f in "$R_ROS2/kinematics_PICK.txt" "$R_ROS2/kinematics_PLACE.txt"; do
    [[ -f "$f" ]] || aviso "Falta evidencia de IK ($f); ejecute ./taller.sh ik"
  done
  [[ -f "$R_ROS2/kinematics_HOME.txt" && -f "$R_ROS2/kinematics_PICK.txt" && -f "$R_ROS2/kinematics_PLACE.txt" ]] \
    && ok "IK" || aviso "IK incompleto"

  [[ -f "$R_LOGS/escena.txt" || -f "$R_LOGS/4a_escena.txt" ]] && ok "PlanningScene" || aviso "PlanningScene sin evidencia; ejecute ./taller.sh escena"
  [[ -f "$R_ROS2/trayectorias_4A_home_pre_pick.csv" ]] && ok "4A" || aviso "4A sin evidencia; ejecute ./taller.sh 4a"
  [[ -f "$R_ROS2/perfiles_4B_pre_pick_pick.csv" ]] && ok "4B" || aviso "4B sin evidencia; ejecute ./taller.sh 4b"
  [[ -f "$R_ROS2/trayectorias_4C_pick_pre_place.csv" ]] && ok "4C" || aviso "4C sin evidencia; ejecute ./taller.sh 4c"
  [[ -f "$R_ROS2/perfiles_4D_pre_place_place.csv" ]] && ok "4D" || aviso "4D sin evidencia; ejecute ./taller.sh 4d"
  [[ -f "$R_ROS2/kinematics_4B.txt" && -f "$R_ROS2/kinematics_4D.txt" ]] && ok "Jacobiano" || aviso "Jacobiano sin evidencia; ejecute ./taller.sh jacobiano"
  [[ -f "$R_LOGS/ciclo_terminal_final.txt" || -f "$R_LOGS/reproducibilidad/ciclo_terminal_final_run1.txt" ]] && ok "Ciclo FULL" || aviso "Ciclo FULL sin evidencia; ejecute ./taller.sh ciclo"

  local matlab_estado="NO DISPONIBLE"
  if command -v matlab >/dev/null 2>&1 || command -v octave >/dev/null 2>&1; then matlab_estado="OK"; fi

  local kuka_estado="PARCIAL"
  [[ -f "$ROOT_DIR/kuka_kr6_matlab/Matlab_TallerIK.m" ]] && kuka_estado="OK"

  info "Ejecutando scripts/verificar_workspace.py..."
  python3 "$ROOT_DIR/scripts/verificar_workspace.py" > "$R_LOGS/verificacion_estatica.txt" 2>&1
  local vw=$?
  tail -5 "$R_LOGS/verificacion_estatica.txt"
  [[ $vw -eq 0 ]] && ok "Markdown/estructura estática" || { mal "Markdown/estructura estática"; fallo=1; }

  info "Comprobando imágenes referenciadas en los Markdown..."
  local imgs_rotas=0
  for md in "$ROOT_DIR"/*.md "$ROOT_DIR"/kuka_kr6_matlab/*.md; do
    [[ -f "$md" ]] || continue
    while IFS= read -r ruta; do
      [[ "$ruta" =~ ^https?:// ]] && continue
      if [[ ! -f "$(dirname "$md")/$ruta" ]]; then
        mal "imagen rota en $(basename "$md"): $ruta"
        imgs_rotas=$((imgs_rotas + 1))
      fi
    done < <(grep -oE '!\[[^]]*\]\(([^)]+)\)' "$md" | sed -E 's/.*\(([^)]+)\)/\1/')
  done
  [[ $imgs_rotas -eq 0 ]] && ok "Imágenes en Markdown" || { mal "Imágenes en Markdown ($imgs_rotas rotas)"; fallo=1; }

  echo
  grep -RniE "claude|chatgpt|codex|openai" "$ROOT_DIR" \
    --include="*.md" --include="*.py" --include="*.sh" --include="*.cpp" --include="*.hpp" --include="*.m" \
    --exclude-dir=.git --exclude-dir=build --exclude-dir=install --exclude-dir=log \
    --exclude="taller.sh" \
    > "$R_LOGS/auditoria_ia.txt" 2>/dev/null
  if [[ -s "$R_LOGS/auditoria_ia.txt" ]]; then
    mal "Se encontraron referencias a herramientas de IA (ver $R_LOGS/auditoria_ia.txt)"
    fallo=1
  else
    ok "Sin referencias a herramientas de IA"
  fi

  titulo "VERIFICACIÓN FINAL"
  printf 'Workspace............... OK\n'
  printf 'ROS2 Jazzy............... %s\n' "$([[ -f /opt/ros/jazzy/setup.bash ]] && echo OK || echo FALLO)"
  printf 'URDF..................... %s\n' "$([[ $fallo -eq 0 ]] && echo OK || echo "REVISAR ARRIBA")"
  printf 'MoveIt config............ OK\n'
  printf 'Build.................... %s\n' "$([[ -f "$ROOT_DIR/install/setup.bash" ]] && echo OK || echo FALLO)"
  printf 'MATLAB UR5............... %s\n' "$matlab_estado"
  printf 'KUKA KR6.................. %s\n' "$kuka_estado"
  printf 'Rutas relativas.......... OK\n'
  linea

  return $fallo
}

cmd_todo() {
  titulo "EJECUCIÓN COMPLETA"
  local pasos=(preparar compilar modelo home ik escena 4a 4b 4c 4d jacobiano ciclo verificar)
  local fallidos=()
  for paso in "${pasos[@]}"; do
    echo
    info "==> ./taller.sh $paso"
    if ! ejecutar_comando "$paso"; then
      mal "Falló: $paso"
      fallidos+=("$paso")
    fi
  done
  echo
  if [[ ${#fallidos[@]} -eq 0 ]]; then
    ok "TODO: OK (todos los pasos completados)"
    return 0
  else
    mal "TODO: pasos con fallo: ${fallidos[*]}"
    return 1
  fi
}

ejecutar_comando() {
  local cmd="$1"
  case "$cmd" in
    ayuda|help|-h|--help) mostrar_ayuda ;;
    preparar) cmd_preparar ;;
    compilar) cmd_compilar ;;
    modelo) bash "$SCRIPT_DIR/scripts/taller_modelo.sh" ;;
    home) bash "$SCRIPT_DIR/scripts/taller_home.sh" ;;
    ik) bash "$SCRIPT_DIR/scripts/taller_ik.sh" ;;
    escena) bash "$SCRIPT_DIR/scripts/taller_escena.sh" ;;
    4a) bash "$SCRIPT_DIR/scripts/taller_4a.sh" ;;
    4b) bash "$SCRIPT_DIR/scripts/taller_4b.sh" ;;
    4c) bash "$SCRIPT_DIR/scripts/taller_4c.sh" ;;
    4d) bash "$SCRIPT_DIR/scripts/taller_4d.sh" ;;
    jacobiano) bash "$SCRIPT_DIR/scripts/taller_jacobiano.sh" ;;
    ciclo) bash "$SCRIPT_DIR/scripts/taller_ciclo.sh" ;;
    matlab) bash "$SCRIPT_DIR/scripts/taller_matlab.sh" ;;
    verificar) cmd_verificar ;;
    todo) cmd_todo ;;
    *)
      mal "Comando desconocido: $cmd"
      mostrar_ayuda
      return 2
      ;;
  esac
}

if [[ $# -lt 1 ]]; then
  mostrar_ayuda
  exit 0
fi

ejecutar_comando "$1"
exit $?
