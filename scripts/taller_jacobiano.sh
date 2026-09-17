#!/usr/bin/env bash
# PARTE 5 - Jacobiano: J_DH vs J_MoveIt/KDL en HOME, PICK, PLACE, 4B y 4D.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/taller_common.sh"

titulo "PARTE 5 — JACOBIANO (J_DH vs J_MoveIt/KDL)"

cargar_ros || exit 1
cd "$ROOT_DIR" || exit 1
asegurar_moveit || exit 1

resultado=0
resumen_csv="$R_IMAGENES/.jacobiano_resumen.csv"
echo "estado,frobenius,tcp_esperado,tcp_obtenido,error_tcp" > "$resumen_csv"

for estado in HOME PICK PLACE 4B 4D; do
  ros2 run ur5_pick_place kinematics_report --ros-args -p state:="$estado" -p results_dir:="$R_ROS2" \
    > "$R_LOGS/jacobiano_${estado,,}_stdout.txt" 2>&1
  archivo="$R_ROS2/kinematics_${estado}.txt"
  if [[ ! -f "$archivo" ]]; then
    mal "No se generó $archivo"
    resultado=1
    continue
  fi

  titulo "JACOBIANO — $estado"
  q=$(sed -n '/Orden de articulaciones/,/^$/p' "$archivo" | grep "^  q")
  j_dh=$(sed -n '/Jacobiano analitico DH/,/^$/p' "$archivo" | tail -n +2)
  j_moveit=$(sed -n '/Jacobiano MoveIt\/KDL/,/^$/p' "$archivo" | tail -n +2)
  frob=$(extraer_valor "$archivo" "Norma Frobenius del error: ")

  echo "q ="
  echo "$q"
  echo
  echo "J_DH ="
  echo "$j_dh"
  echo
  echo "J_MoveIt ="
  echo "$j_moveit"
  echo
  echo "||J_DH - J_MoveIt||F = $frob"

  tcp_esp=""; tcp_obt=""; err_tcp=""
  if [[ "$estado" == "4B" || "$estado" == "4D" ]]; then
    qdot=$(grep -A1 "^qdot \[rad/s\]:" "$archivo" | tail -1)
    xdot=$(grep -A1 "^xdot MoveIt" "$archivo" | tail -1)
    tcp_esp=$([[ "$estado" == "4B" ]] && echo "0.200" || echo "0.100")
    xdot_exigida=$(grep -A1 "^xdot exigida por perfil" "$archivo" | tail -1)
    tcp_obt=$(extraer_valor "$archivo" "Error de velocidad cartesiana: ")
    limites=$(grep "Limites de velocidad articular:" "$archivo" | tail -1)
    echo
    echo "qdot [rad/s] ="
    echo "$qdot"
    echo "xdot obtenida ="
    echo "$xdot"
    echo "xdot exigida por el perfil ="
    echo "$xdot_exigida"
    echo "TCP esperado = $tcp_esp m/s"
    echo "Error de velocidad cartesiana = $tcp_obt"
    echo "$limites"
    err_tcp="$tcp_obt"
  fi

  if cumple_umbral "$frob" 0.0001; then
    ok "RESULTADO: CUMPLE"
  else
    mal "RESULTADO: NO CUMPLE"
    resultado=1
  fi
  echo "$estado,$frob,$tcp_esp,$tcp_obt,$err_tcp" >> "$resumen_csv"
  echo
done

info "Generando gráfica de error de Jacobiano por estado..."
python3 - "$resumen_csv" "$R_IMAGENES/jacobiano_error.png" <<'PYEOF'
import csv, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

entrada, salida = sys.argv[1], sys.argv[2]
filas = list(csv.DictReader(open(entrada, encoding="utf-8")))
estados = [f["estado"] for f in filas]
frob = [float(f["frobenius"]) if f["frobenius"] else 0.0 for f in filas]

fig, ax = plt.subplots(figsize=(7, 4.5))
ax.bar(estados, frob, color="#1f77b4")
ax.set_yscale("log")
ax.set_title("Error de Jacobiano (DH vs MoveIt/KDL) por estado")
ax.set_xlabel("Estado")
ax.set_ylabel("||J_DH - J_MoveIt||_F  (escala log)")
ax.grid(True, axis="y", alpha=0.3)
for i, v in enumerate(frob):
    ax.text(i, v, f"{v:.1e}", ha="center", va="bottom", fontsize=8)
fig.tight_layout()
fig.savefig(salida, dpi=140)
print(f"Gráfica guardada en: {salida}")
PYEOF

rm -f "$resumen_csv"

echo
echo "Evidencia: resultados/ros2/kinematics_{HOME,PICK,PLACE,4B,4D}.{txt,csv}"
echo "           $R_IMAGENES/jacobiano_error.png"

[[ $resultado -eq 0 ]] && ok "PARTE 5: OK" || mal "PARTE 5: FALLÓ"
exit $resultado
