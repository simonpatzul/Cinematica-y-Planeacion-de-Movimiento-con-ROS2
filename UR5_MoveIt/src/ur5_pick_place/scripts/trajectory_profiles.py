#!/usr/bin/env python3
"""Genera y compara perfiles escalares cubico y quintico para 4B y 4D.

El script no mueve el robot. Produce referencias reproducibles de posicion,
velocidad y aceleracion que sirven para estudiar la parametrizacion temporal
de las trayectorias cartesianas creadas por MoveIt.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


@dataclass(frozen=True)
class Segmento:
    nombre: str
    color: str
    distancia_m: float
    velocidad_max_m_s: float
    aceleracion_max_m_s2: float


SEGMENTOS = (
    Segmento("4B_rojo", "red", 0.33, 0.200, 0.300),
    Segmento("4D_azul", "blue", 0.33, 0.100, 0.020),
)


def evaluar_perfil(nombre: str, tau: float) -> tuple[float, float, float]:
    """Devuelve s, ds/dtau y d2s/dtau2 para tau en [0, 1]."""
    if nombre == "cubico":
        return 3 * tau**2 - 2 * tau**3, 6 * tau - 6 * tau**2, 6 - 12 * tau
    if nombre == "quintico":
        return (
            10 * tau**3 - 15 * tau**4 + 6 * tau**5,
            30 * tau**2 - 60 * tau**3 + 30 * tau**4,
            60 * tau - 180 * tau**2 + 120 * tau**3,
        )
    raise ValueError(f"Perfil desconocido: {nombre}")


def factores_maximos(nombre: str, muestras: int = 100_001) -> tuple[float, float]:
    """Obtiene numericamente los maximos normalizados de velocidad y aceleracion."""
    max_v = 0.0
    max_a = 0.0
    for i in range(muestras):
        tau = i / (muestras - 1)
        _, ds, dds = evaluar_perfil(nombre, tau)
        max_v = max(max_v, abs(ds))
        max_a = max(max_a, abs(dds))
    return max_v, max_a


def duracion_minima(segmento: Segmento, perfil: str) -> float:
    """Calcula T para cumplir simultaneamente los limites de v y a."""
    factor_v, factor_a = factores_maximos(perfil)
    por_velocidad = segmento.distancia_m * factor_v / segmento.velocidad_max_m_s
    por_aceleracion = math.sqrt(
        segmento.distancia_m * factor_a / segmento.aceleracion_max_m_s2
    )
    # Margen pequeno para evitar falsos incumplimientos por redondeo numerico.
    return 1.001 * max(por_velocidad, por_aceleracion)


def generar_muestras(segmento: Segmento, perfil: str, cantidad: int = 201) -> list[dict]:
    duracion = duracion_minima(segmento, perfil)
    filas = []
    for i in range(cantidad):
        tau = i / (cantidad - 1)
        s, ds, dds = evaluar_perfil(perfil, tau)
        filas.append(
            {
                "tiempo_s": tau * duracion,
                "tau": tau,
                "progreso": s,
                "posicion_m": segmento.distancia_m * s,
                "velocidad_m_s": segmento.distancia_m * ds / duracion,
                "aceleracion_m_s2": segmento.distancia_m * dds / duracion**2,
            }
        )
    return filas


def escribir_csv(ruta: Path, filas: list[dict]) -> None:
    with ruta.open("w", newline="", encoding="utf-8") as archivo:
        escritor = csv.DictWriter(archivo, fieldnames=filas[0].keys())
        escritor.writeheader()
        escritor.writerows(filas)


def escribir_waypoints(ruta: Path, segmento: Segmento, perfil: str) -> None:
    duracion = duracion_minima(segmento, perfil)
    filas = []
    for indice, tau in enumerate((0.0, 0.2, 0.4, 0.6, 0.8, 1.0)):
        s, _, _ = evaluar_perfil(perfil, tau)
        filas.append(
            {
                "waypoint": indice,
                "tiempo_s": tau * duracion,
                "progreso": s,
                "desplazamiento_m": segmento.distancia_m * s,
            }
        )
    escribir_csv(ruta, filas)


def graficar(ruta: Path, segmento: Segmento, datos: dict[str, list[dict]]) -> None:
    figura, ejes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
    for perfil, filas in datos.items():
        tiempos = [fila["tiempo_s"] for fila in filas]
        ejes[0].plot(tiempos, [f["posicion_m"] for f in filas], label=perfil)
        ejes[1].plot(tiempos, [f["velocidad_m_s"] for f in filas], label=perfil)
        ejes[2].plot(tiempos, [f["aceleracion_m_s2"] for f in filas], label=perfil)
    ejes[0].set_ylabel("Posicion [m]")
    ejes[1].set_ylabel("Velocidad [m/s]")
    ejes[2].set_ylabel("Aceleracion [m/s²]")
    ejes[2].set_xlabel("Tiempo [s]")
    ejes[1].axhline(segmento.velocidad_max_m_s, color=segmento.color, linestyle="--")
    ejes[1].axhline(-segmento.velocidad_max_m_s, color=segmento.color, linestyle="--")
    ejes[2].axhline(segmento.aceleracion_max_m_s2, color=segmento.color, linestyle="--")
    ejes[2].axhline(-segmento.aceleracion_max_m_s2, color=segmento.color, linestyle="--")
    for eje in ejes:
        eje.grid(True, alpha=0.3)
        eje.legend()
    figura.suptitle(f"{segmento.nombre}: perfiles cubico y quintico")
    figura.tight_layout()
    figura.savefig(ruta, dpi=160)
    plt.close(figura)


def ejecutar(directorio: Path) -> None:
    directorio.mkdir(parents=True, exist_ok=True)
    resumen = []
    for segmento in SEGMENTOS:
        datos = {}
        for perfil in ("cubico", "quintico"):
            filas = generar_muestras(segmento, perfil)
            datos[perfil] = filas
            escribir_csv(directorio / f"perfil_{segmento.nombre}_{perfil}.csv", filas)
            escribir_waypoints(
                directorio / f"waypoints_{segmento.nombre}_{perfil}.csv", segmento, perfil
            )
            vmax = max(abs(f["velocidad_m_s"]) for f in filas)
            amax = max(abs(f["aceleracion_m_s2"]) for f in filas)
            # Indicador sencillo de suavidad: variacion total de aceleracion.
            # Menor valor significa cambios de aceleracion mas suaves.
            variacion_aceleracion = (
                abs(filas[0]["aceleracion_m_s2"])
                + sum(
                    abs(filas[i]["aceleracion_m_s2"] - filas[i - 1]["aceleracion_m_s2"])
                    for i in range(1, len(filas))
                )
                + abs(filas[-1]["aceleracion_m_s2"])
            )
            resumen.append(
                {
                    "tramo": segmento.nombre,
                    "perfil": perfil,
                    "distancia_m": segmento.distancia_m,
                    "duracion_s": filas[-1]["tiempo_s"],
                    "velocidad_max_observada_m_s": vmax,
                    "limite_velocidad_m_s": segmento.velocidad_max_m_s,
                    "aceleracion_max_observada_m_s2": amax,
                    "limite_aceleracion_m_s2": segmento.aceleracion_max_m_s2,
                    "variacion_aceleracion_m_s2": variacion_aceleracion,
                    "cumple_limites": vmax <= segmento.velocidad_max_m_s + 1e-9
                    and amax <= segmento.aceleracion_max_m_s2 + 1e-9,
                }
            )
        graficar(directorio / f"comparacion_perfiles_{segmento.nombre}.png", segmento, datos)
    escribir_csv(directorio / "resumen_perfiles.csv", resumen)
    print(f"Resultados escritos en: {directorio.resolve()}")
    for fila in resumen:
        print(
            f"{fila['tramo']:8s} {fila['perfil']:8s} "
            f"T={fila['duracion_s']:.3f}s "
            f"vmax={fila['velocidad_max_observada_m_s']:.4f}m/s "
            f"amax={fila['aceleracion_max_observada_m_s2']:.4f}m/s² "
            f"var_a={fila['variacion_aceleracion_m_s2']:.4f} "
            f"cumple={fila['cumple_limites']}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "resultados",
        help="Directorio donde se escriben CSV y PNG",
    )
    argumentos = parser.parse_args()
    ejecutar(argumentos.output_dir)


if __name__ == "__main__":
    main()
