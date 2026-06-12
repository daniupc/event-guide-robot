#!/usr/bin/env python3

"""Generate Gazebo marker textures for the semantic stands.

The embedded bytes are the first 50 markers from OpenCV's DICT_4X4_50
predefined dictionary. They are stored as the rotation-0 row-major bytes used
by OpenCV's aruco module, so the generated PNGs match vision_detector_node.py.
"""

import re
import struct
import zlib
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SEMANTIC_MAP = PACKAGE_ROOT / "config" / "semantic_map.yaml"
TEXTURE_DIR = PACKAGE_ROOT / "media" / "materials" / "textures"
MATERIAL_FILE = PACKAGE_ROOT / "media" / "materials" / "scripts" / "event_guide_markers.material"

DICT_4X4_50_BYTES = {
    0: (181, 50),
    1: (15, 154),
    2: (51, 45),
    3: (153, 70),
    4: (84, 158),
    5: (121, 205),
    6: (158, 46),
    7: (196, 242),
    8: (254, 218),
    9: (207, 86),
    10: (249, 145),
    11: (17, 167),
    12: (14, 183),
    13: (42, 15),
    14: (36, 177),
    15: (38, 62),
    16: (70, 101),
    17: (102, 0),
    18: (108, 94),
    19: (118, 175),
    20: (134, 139),
    21: (176, 43),
    22: (204, 213),
    23: (221, 130),
    24: (254, 71),
    25: (148, 113),
    26: (172, 228),
    27: (165, 84),
    28: (33, 35),
    29: (52, 111),
    30: (68, 21),
    31: (87, 178),
    32: (158, 207),
    33: (240, 203),
    34: (8, 174),
    35: (9, 41),
    36: (24, 117),
    37: (4, 255),
    38: (13, 246),
    39: (28, 90),
    40: (23, 24),
    41: (42, 40),
    42: (50, 140),
    43: (56, 178),
    44: (36, 232),
    45: (46, 235),
    46: (45, 63),
    47: (75, 100),
    48: (80, 46),
    49: (80, 19),
}


def semantic_marker_ids():
    text = SEMANTIC_MAP.read_text(encoding="utf-8")
    return sorted({int(match) for match in re.findall(r"marker_id:\s*(\d+)", text)})


def marker_bits(marker_id):
    if marker_id not in DICT_4X4_50_BYTES:
        raise ValueError(f"marker_id {marker_id} is outside DICT_4X4_50")

    values = DICT_4X4_50_BYTES[marker_id]
    bits = []
    for value in values:
        for mask in (128, 64, 32, 16, 8, 4, 2, 1):
            bits.append(1 if value & mask else 0)
    return [bits[row * 4:(row + 1) * 4] for row in range(4)]


def marker_pixels(marker_id, side=512, quiet_ratio=0.14):
    quiet = int(side * quiet_ratio)
    marker_side = side - quiet * 2
    cell = marker_side // 6
    start = (side - cell * 6) // 2
    pixels = bytearray([255] * side * side)
    bits = marker_bits(marker_id)

    def fill_cell(row, col, color):
        y0 = start + row * cell
        x0 = start + col * cell
        for y in range(y0, y0 + cell):
            offset = y * side + x0
            pixels[offset:offset + cell] = bytes([color]) * cell

    for row in range(6):
        for col in range(6):
            if row == 0 or row == 5 or col == 0 or col == 5:
                fill_cell(row, col, 0)
            else:
                fill_cell(row, col, 255 if bits[row - 1][col - 1] else 0)

    return side, side, bytes(pixels)


def write_png(path, width, height, gray_pixels):
    def chunk(kind, data):
        payload = kind + data
        return struct.pack(">I", len(data)) + payload + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF)

    rows = bytearray()
    for y in range(height):
        rows.append(0)
        rows.extend(gray_pixels[y * width:(y + 1) * width])

    data = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(bytes(rows), 9))
        + chunk(b"IEND", b"")
    )
    path.write_bytes(data)


def write_material_file(marker_ids):
    blocks = []
    for marker_id in marker_ids:
        blocks.append(
            f"""material EventGuide/Aruco{marker_id}
{{
  technique
  {{
    pass
    {{
      lighting off
      ambient 1 1 1 1
      diffuse 1 1 1 1
      texture_unit
      {{
        texture aruco_{marker_id}.png
        filtering none
      }}
    }}
  }}
}}
"""
        )
    MATERIAL_FILE.write_text("\n".join(blocks), encoding="utf-8")


def main():
    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
    MATERIAL_FILE.parent.mkdir(parents=True, exist_ok=True)

    marker_ids = semantic_marker_ids()
    for marker_id in marker_ids:
        width, height, pixels = marker_pixels(marker_id)
        write_png(TEXTURE_DIR / f"aruco_{marker_id}.png", width, height, pixels)
    write_material_file(marker_ids)

    print(f"Generated {len(marker_ids)} Gazebo marker textures in {TEXTURE_DIR}")


if __name__ == "__main__":
    main()
