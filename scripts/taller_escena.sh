#!/usr/bin/env bash
# PARTE 4.0 - PlanningScene: aplica y verifica los objetos reales de la escena.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "PARTE 4.0 — PLANNING SCENE"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1
asegurar_moveit || exit 1

info "Aplicando escena (pick_surface, place_surface, workpiece, obstacle)..."
recargar_escena > "$R_LOGS/escena.txt" 2>&1
cat "$R_LOGS/escena.txt"

info "Consultando /get_planning_scene para confirmar los objetos..."
salida=$(timeout 10 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 24}}" 2>/dev/null)

esperados=(pick_surface place_surface workpiece obstacle)
encontrados=0
echo
for obj in "${esperados[@]}"; do
  if echo "$salida" | grep -q "id='$obj'"; then
    ok "objeto presente: $obj"
    encontrados=$((encontrados + 1))
  else
    mal "objeto ausente: $obj"
  fi
done

echo
echo "Objetos esperados: ${#esperados[@]}"
echo "Objetos encontrados: $encontrados"

if [[ $encontrados -eq ${#esperados[@]} ]]; then
  ok "PlanningScene: OK"
  resultado=0
else
  mal "PlanningScene: INCOMPLETA"
  resultado=1
fi

echo
echo "Evidencia: $R_LOGS/escena.txt"
if [[ -n "${DISPLAY:-}" ]]; then
  info "Capturando ventana de RViz (xwd)..."
  win_id=$(xwininfo -root -tree 2>/dev/null | grep -oE '0x[0-9a-f]+ +"[^"]*RViz"' | grep -oE '^0x[0-9a-f]+' | head -1)
  if [[ -n "$win_id" ]]; then
    xwd -id "$win_id" -silent -out /tmp/taller_escena.xwd 2>/dev/null
    python3 "$COMMON_DIR/xwd_a_png.py" /tmp/taller_escena.xwd "$R_IMAGENES/planning_scene.png" 2>/dev/null \
      && ok "Captura guardada en $R_IMAGENES/planning_scene.png" \
      || aviso "No se pudo convertir la captura de RViz a PNG."
  else
    aviso "No se encontró la ventana de RViz para capturar."
  fi
fi

exit $resultado
