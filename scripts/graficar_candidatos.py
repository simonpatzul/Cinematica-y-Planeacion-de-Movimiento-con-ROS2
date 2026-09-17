#!/usr/bin/env python3
"""Lee un CSV de candidatos (trayectorias_4A_*.csv / trayectorias_4C_*.csv) real
generado por pick_place_sequence y produce:
  - una tabla resumida en la terminal (sin volcar los 20 logs de ROS);
  - una grafica PNG con tiempo de planeacion y longitud articular por intento.

No inventa datos: todo sale del CSV que escribio el ejecutable C++.

Uso: graficar_candidatos.py entrada.csv salida.png "Titulo de la grafica"
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main() -> int:
    if len(sys.argv) != 4:
        print("Uso: graficar_candidatos.py entrada.csv salida.png titulo", file=sys.stderr)
        return 2

    entrada, salida, titulo = sys.argv[1], sys.argv[2], sys.argv[3]
    filas = list(csv.DictReader(open(entrada, encoding="utf-8")))
    if not filas:
        print("CSV vacío, no se genera gráfica.", file=sys.stderr)
        return 1

    print(f"{'Planeador':<24} {'Intento':>7} {'Éxito':>6} {'t_plan[s]':>10} "
          f"{'Longitud[rad]':>14} {'Puntos':>7} {'Duración[s]':>12} {'Suavidad':>10}")
    for f in filas:
        exito = "SI" if f["valida"] == "si" else "NO"
        print(f"{f['planeador']:<24} {f['intento']:>7} {exito:>6} "
              f"{float(f['tiempo_planificacion_s']):>10.4f} "
              f"{float(f['longitud_articular_rad']):>14.5f} {f['puntos']:>7} "
              f"{float(f['duracion_s']):>12.3f} {float(f['suavidad']):>10.4f}")

    planeadores = sorted({f["planeador"] for f in filas})
    colores = {p: c for p, c in zip(planeadores, ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"])}

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
    for p in planeadores:
        xs = [int(f["intento"]) for f in filas if f["planeador"] == p]
        t_plan = [float(f["tiempo_planificacion_s"]) for f in filas if f["planeador"] == p]
        longitud = [
            float(f["longitud_articular_rad"]) if f["valida"] == "si" else float("nan")
            for f in filas if f["planeador"] == p
        ]
        validos = [f["valida"] == "si" for f in filas if f["planeador"] == p]
        marcadores = ["o" if v else "x" for v in validos]

        ax1.plot(xs, t_plan, "-", color=colores[p], label=p, linewidth=1.5)
        for x, y, m in zip(xs, t_plan, marcadores):
            ax1.scatter(x, y, color=colores[p], marker=m, zorder=3)

        ax2.plot(xs, longitud, "-o", color=colores[p], label=p, linewidth=1.5)

    ax1.set_title("Tiempo de planeación por intento")
    ax1.set_xlabel("Intento")
    ax1.set_ylabel("Tiempo de planeación [s]")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2.set_title("Longitud articular por intento (válidos)")
    ax2.set_xlabel("Intento")
    ax2.set_ylabel("Longitud articular [rad]")
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    fig.suptitle(titulo)
    fig.tight_layout()
    fig.savefig(salida, dpi=140)
    print(f"\nGráfica guardada en: {salida}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
