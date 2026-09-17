#!/usr/bin/env bash
# Ejecuta la validacion DH del UR5 en MATLAB u Octave (sin Robotics System Toolbox)
# y genera las graficas de evidencia a partir de los CSV/TXT reales de ROS2.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "VALIDACIÓN MATLAB DEL UR5"

MOTOR=""
if command -v matlab >/dev/null 2>&1; then
  MOTOR="matlab"
elif command -v octave >/dev/null 2>&1; then
  MOTOR="octave"
fi

if [[ -z "$MOTOR" ]]; then
  aviso "MATLAB/Octave NO DISPONIBLE en este equipo. No se ejecuta nada (no se inventan resultados)."
  echo "Scripts que se ejecutarían: matlab/UR5_DH_VALIDACION.m, matlab/generar_evidencia_grafica.m"
  exit 1
fi
ok "Motor disponible: $MOTOR"

cd "$ROOT_DIR/matlab" || exit 1

info "Ejecutando UR5_DH_VALIDACION.m (HOME/PICK/PLACE, sin toolbox)..."
if [[ "$MOTOR" == "matlab" ]]; then
  matlab -batch "UR5_DH_VALIDACION" > "$R_LOGS/matlab_UR5_DH_VALIDACION.txt" 2>&1
else
  octave --no-gui --eval "UR5_DH_VALIDACION" > "$R_LOGS/matlab_UR5_DH_VALIDACION.txt" 2>&1
fi
codigo1=$?
cat "$R_LOGS/matlab_UR5_DH_VALIDACION.txt"

echo
info "Generando gráficas de evidencia (01 a 08) desde los CSV/TXT reales..."
if [[ "$MOTOR" == "matlab" ]]; then
  matlab -batch "generar_evidencia_grafica" > "$R_LOGS/matlab_evidencia_grafica.txt" 2>&1
else
  octave --no-gui --eval "generar_evidencia_grafica" > "$R_LOGS/matlab_evidencia_grafica.txt" 2>&1
fi
codigo2=$?
cat "$R_LOGS/matlab_evidencia_grafica.txt"

echo
echo "Imágenes generadas en $R_MATLAB:"
ls -la "$R_MATLAB"/*.png 2>/dev/null || aviso "No se generó ninguna imagen."

echo
echo "Evidencia:"
echo "  $R_LOGS/matlab_UR5_DH_VALIDACION.txt"
echo "  $R_LOGS/matlab_evidencia_grafica.txt"
echo "  $R_MATLAB/*.png"

if [[ $codigo1 -eq 0 && $codigo2 -eq 0 ]]; then
  ok "MATLAB UR5: OK"
  exit 0
else
  mal "MATLAB UR5: FALLÓ"
  exit 1
fi
