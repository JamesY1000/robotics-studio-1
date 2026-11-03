#!/usr/bin/env python3
"""Extract every non-tree obstacle (bush, rock) from an SDF world into a CSV.

Trees have their own extractor (``extract_tree_locations.py``). This script
covers the remaining ground clutter the dashboard plots as ground truth:

    name, obstacle_type, x, y, z, roll, pitch, yaw

* ``obstacle_type`` is the top-level model folder, lower-cased and singularised:
  ``model://bushes/bush_3`` -> ``bush``, ``model://rocks/rock1`` -> ``rock``.
* ``x, y, z`` (and ``roll, pitch, yaw``) come from the block's ``<pose>``.

Commented-out ``<include>`` blocks are ignored (ElementTree drops XML comments),
so obstacles disabled in the world do not appear in the output.

Usage
-----
    python extract_obstacle_locations.py                  # default world -> default csv
    python extract_obstacle_locations.py world.sdf out.csv
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# Repo-relative defaults: <repo>/rs1_ws/src/rs1_sim/{worlds,scripts}
SIM_DIR = Path(__file__).resolve().parent.parent
DEFAULT_WORLD = SIM_DIR / "worlds" / "v2_forest.sdf"
DEFAULT_CSV = (SIM_DIR.parent / "rs1_mission" / "res" / "obstacle_locations.csv")

# Ground clutter that is not a tree. Trees are handled by the tree extractor so
# the two inventories never double-count the same model.
OBSTACLE_URI_RE = re.compile(r"model://([^/]*)/([^/]+)", re.IGNORECASE)
# Explicit folder -> type map: naive singularisation gives "bushe" from "bushes".
OBSTACLE_FOLDERS = {
    "bush": "bush", "bushes": "bush",
    "rock": "rock", "rocks": "rock",
}

CSV_HEADER = ["name", "obstacle_type", "x", "y", "z", "roll", "pitch", "yaw"]


def parse_pose(element: ET.Element | None) -> list[float]:
    """Return a 6-component pose, padding missing values with 0.0."""
    if element is None or not (element.text or "").strip():
        return [0.0] * 6
    values = [float(v) for v in element.text.split()]
    return (values + [0.0] * 6)[:6]


def obstacle_type_from_uri(uri: str) -> str | None:
    """Return the singular lower-case obstacle type for a model URI."""
    match = OBSTACLE_URI_RE.search(uri)
    if not match:
        return None
    folder = match.group(1).lower()
    return OBSTACLE_FOLDERS.get(folder)


def extract_obstacles(world_path: Path) -> list[dict[str, object]]:
    """Parse ``world_path`` and return one dict per bush/rock include block."""
    root = ET.parse(world_path).getroot()

    obstacles: list[dict[str, object]] = []
    for include in root.iter("include"):
        uri_el = include.find("uri")
        if uri_el is None:
            continue

        obstacle_type = obstacle_type_from_uri(uri_el.text or "")
        if obstacle_type is None:
            continue

        name_el = include.find("name")
        name = name_el.text.strip() if name_el is not None and name_el.text else ""
        x, y, z, roll, pitch, yaw = parse_pose(include.find("pose"))

        obstacles.append(
            {
                "name": name,
                "obstacle_type": obstacle_type,
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
        )
    return obstacles


def write_csv(obstacles: list[dict[str, object]], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_HEADER)
        writer.writeheader()
        writer.writerows(obstacles)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("world", nargs="?", type=Path, default=DEFAULT_WORLD,
                        help=f"SDF world file (default: {DEFAULT_WORLD})")
    parser.add_argument("csv", nargs="?", type=Path, default=DEFAULT_CSV,
                        help=f"CSV file to write (default: {DEFAULT_CSV})")
    args = parser.parse_args(argv)

    world = args.world.expanduser().resolve()
    if not world.is_file():
        print(f"error: world file not found: {world}", file=sys.stderr)
        return 2

    obstacles = extract_obstacles(world)
    write_csv(obstacles, args.csv)
    print(f"wrote {len(obstacles)} obstacle(s) from {world} to {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
