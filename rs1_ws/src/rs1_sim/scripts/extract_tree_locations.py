#!/usr/bin/env python3
"""Extract every tree from a Gazebo/SDF world file into a CSV table.

Each ``<include>`` block that references a tree model (``model://trees/<type>_tree``)
is written as one CSV row containing:

    name, tree_type, x, y, z, roll, pitch, yaw

* ``tree_type`` is the model name with the ``_tree`` suffix stripped and
  lower-cased, e.g. ``model://trees/pine_tree`` -> ``pine``.
* ``x, y, z`` (and ``roll, pitch, yaw``) come from the block's ``<pose>``.

Commented-out ``<include>`` blocks are ignored (ElementTree drops XML comments),
so trees that are disabled in the world do not appear in the output.

Usage
-----
    python extract_tree_locations.py                      # default world -> default csv
    python extract_tree_locations.py world.sdf out.csv
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
DEFAULT_CSV = Path(__file__).resolve().parent / "tree_locations.csv"

# Matches model URIs such as "model://trees/oak_tree" and captures "oak".
TREE_URI_RE = re.compile(r"model://[^/]*trees?/([^/]+)", re.IGNORECASE)

CSV_HEADER = ["name", "tree_type", "x", "y", "z", "roll", "pitch", "yaw"]


def parse_pose(element: ET.Element | None) -> list[float]:
    """Return a 6-component pose, padding missing values with 0.0."""
    if element is None or not (element.text or "").strip():
        return [0.0] * 6
    values = [float(v) for v in element.text.split()]
    return (values + [0.0] * 6)[:6]


def tree_type_from_uri(uri: str) -> str | None:
    """Return the lower-case tree type for a tree model URI, else ``None``."""
    match = TREE_URI_RE.search(uri)
    if not match:
        return None
    model = match.group(1)
    return re.sub(r"_tree$", "", model, flags=re.IGNORECASE).lower()


def extract_trees(world_path: Path) -> list[dict[str, object]]:
    """Parse ``world_path`` and return one dict per tree include block."""
    root = ET.parse(world_path).getroot()

    trees: list[dict[str, object]] = []
    for include in root.iter("include"):
        uri_el = include.find("uri")
        if uri_el is None:
            continue

        tree_type = tree_type_from_uri(uri_el.text or "")
        if tree_type is None:
            continue

        name_el = include.find("name")
        name = name_el.text.strip() if name_el is not None and name_el.text else ""
        x, y, z, roll, pitch, yaw = parse_pose(include.find("pose"))

        trees.append(
            {
                "name": name,
                "tree_type": tree_type,
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
        )
    return trees


def write_csv(trees: list[dict[str, object]], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_HEADER)
        writer.writeheader()
        writer.writerows(trees)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "world",
        nargs="?",
        type=Path,
        default=DEFAULT_WORLD,
        help=f"SDF world file to read (default: {DEFAULT_WORLD})",
    )
    parser.add_argument(
        "csv",
        nargs="?",
        type=Path,
        default=DEFAULT_CSV,
        help=f"CSV file to write (default: {DEFAULT_CSV})",
    )
    args = parser.parse_args(argv)

    if not args.world.is_file():
        parser.error(f"world file not found: {args.world}")

    trees = extract_trees(args.world)
    write_csv(trees, args.csv)

    counts: dict[str, int] = {}
    for tree in trees:
        key = str(tree["tree_type"])
        counts[key] = counts.get(key, 0) + 1
    summary = ", ".join(f"{k}={v}" for k, v in sorted(counts.items())) or "none"
    print(f"Wrote {len(trees)} trees ({summary}) to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
