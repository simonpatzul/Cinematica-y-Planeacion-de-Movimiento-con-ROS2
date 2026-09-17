#!/usr/bin/env python3
"""Convierte una captura X Window Dump (xwd) a PNG sin depender de ImageMagick.

Uso: xwd_a_png.py entrada.xwd salida.png
"""
import struct
import sys

from PIL import Image

CAMPOS = [
    "header_size", "file_version", "pixmap_format", "pixmap_depth", "pixmap_width",
    "pixmap_height", "xoffset", "byte_order", "bitmap_unit", "bitmap_bit_order",
    "bitmap_pad", "bits_per_pixel", "bytes_per_line", "visual_class", "red_mask",
    "green_mask", "blue_mask", "bits_per_rgb", "colormap_entries", "ncolors",
    "window_width", "window_height", "window_x", "window_y", "window_bdrwidth",
]


def convertir(entrada: str, salida: str) -> None:
    with open(entrada, "rb") as f:
        datos = f.read()

    info = dict(zip(CAMPOS, struct.unpack(">25I", datos[:100])))
    offset_pixeles = info["header_size"] + info["ncolors"] * 12
    pixeles = datos[offset_pixeles:]

    modo_bruto = "BGRX" if info["byte_order"] == 0 else "XRGB"
    imagen = Image.frombuffer(
        "RGB",
        (info["pixmap_width"], info["pixmap_height"]),
        pixeles,
        "raw",
        modo_bruto,
        info["bytes_per_line"],
        1,
    )
    imagen.save(salida)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Uso: xwd_a_png.py entrada.xwd salida.png", file=sys.stderr)
        sys.exit(2)
    convertir(sys.argv[1], sys.argv[2])
