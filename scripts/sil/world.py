"""Generate a metric warehouse and matching grid/occupancy maps from one geometry."""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

STARTS = [(-3.0, -2.0), (3.0, 0.0), (-3.0, 2.0)]
DELIVERIES = [(3.0, -2.0), (-3.0, 0.0), (3.0, 2.0)]
WALLS = [
    (0.0, -3.5, 9.0, 0.2),
    (0.0, 3.5, 9.0, 0.2),
    (-4.5, 0.0, 0.2, 7.0),
    (4.5, 0.0, 0.2, 7.0),
    (0.0, -2.125, 0.2, 2.75),
    (0.0, 2.125, 0.2, 2.75),
]


def box(name, x, y, sx, sy):
    return ET.fromstring(
        f'''<model name="{name}"><static>true</static><pose>{x} {y} .5 0 0 0</pose><link name="wall"><collision name="collision"><geometry><box><size>{sx} {sy} 1</size></box></geometry></collision><visual name="visual"><geometry><box><size>{sx} {sy} 1</size></box></geometry></visual></link></model>'''
    )


def cell(x, y):
    return round(y + 3) * 9 + round(x + 4)


def xy(c):
    return (float(c % 9 - 4), float(c // 9 - 3))


def write_maps(out):
    out = Path(out)
    width, height, res = 200, 160, 0.05
    ox, oy = -5.0, -4.0
    image = bytearray()
    for r in range(height):
        y = oy + (height - r - 0.5) * res
        for col in range(width):
            x = ox + (col + 0.5) * res
            image.append(
                0
                if any(
                    abs(x - bx) <= sx / 2 and abs(y - by) <= sy / 2
                    for bx, by, sx, sy in WALLS
                )
                else 254
            )
    (out / "map.pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode() + image)
    (out / "map.yaml").write_text(
        yaml.safe_dump(
            {
                "image": "map.pgm",
                "resolution": res,
                "origin": [ox, oy, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            }
        )
    )
    (out / "grid.map").write_text(
        "\n".join("....#...." if y != 3 else "........." for y in range(7)) + "\n"
    )
