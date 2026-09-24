#!/usr/bin/env python3
"""Prepare the tree-detection samples for YOLO training.

The sample generator (``sample_generator/.../camera_collector.py``) writes YOLO
labels using the **1-based** class ids from the ObjectLabelPlugin
``<classification>`` (1 = OAK_TREE, 2 = PINE_TREE). Ultralytics YOLO expects
**0-based** class ids, so the raw database is not directly usable.

This script builds a clean, self-contained dataset under ``datasets/trees/``:

    * reads ``database/camera_samples/sample_database.csv``
    * skips any image listed in ``database/excluded.csv`` (manual review list)
    * validates every remaining image/label pair
    * splits the samples into train/val (deterministic seed)
    * copies the images and rewrites the labels with ids remapped 1 -> 0, 2 -> 1
    * writes ``data.yaml`` pointing at the prepared dataset

The raw ``database/`` folder is never modified.

``database/excluded.csv`` holds one relative image path per row (e.g.
``images/000500.jpg``); add or remove rows after a manual review to control
which samples end up in the dataset.

Usage:
    python prepare_dataset.py
"""

from __future__ import annotations

import csv
import random
import shutil
from collections import Counter
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "database" / "camera_samples"
SRC_CSV = SRC / "sample_database.csv"
EXCLUDED_CSV = ROOT / "database" / "excluded.csv"
OUT = ROOT / "datasets" / "trees"

# 1-based classification id (as written by the sample generator) -> 0-based YOLO index
CLASS_REMAP = {1: 0, 2: 1}
# 0-based YOLO index -> human readable class name
CLASS_NAMES = {0: "OAK_TREE", 1: "PINE_TREE"}

VAL_FRACTION = 0.15
SEED = 42


def parse_label(label_path: Path) -> list[list[str]]:
    """Parse a YOLO label file, remap class ids and validate coordinates."""
    lines_out: list[list[str]] = []
    with open(label_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 5:
                raise ValueError(
                    f"{label_path}: expected 5 columns (class cx cy w h), got {len(parts)}"
                )
            cls = int(parts[0])
            if cls not in CLASS_REMAP:
                raise ValueError(f"{label_path}: unknown class id {cls} (expected {sorted(CLASS_REMAP)})")
            coords = [float(x) for x in parts[1:]]
            if any(c < 0.0 or c > 1.0 for c in coords):
                raise ValueError(f"{label_path}: normalised coordinate out of [0, 1]: {parts}")
            lines_out.append([str(CLASS_REMAP[cls]), *[f"{c:.6f}" for c in coords]])
    return lines_out


def load_excluded(csv_path: Path) -> set[str]:
    """Return the set of image paths that must be kept out of the dataset.

    Accepts a one-column CSV (header ``ImageFile``) or a plain list of paths. A
    missing file means nothing is excluded.
    """
    if not csv_path.is_file():
        return set()
    excluded: set[str] = set()
    with open(csv_path, newline="") as f:
        for line in f:
            value = line.split(",")[0].strip()
            if not value or value.lower() == "imagefile":
                continue
            excluded.add(value)
    return excluded


def main() -> None:
    if not SRC_CSV.is_file():
        raise SystemExit(f"sample database not found: {SRC_CSV}")

    excluded = load_excluded(EXCLUDED_CSV)

    # Load sample list from the CSV (skipping anything on the exclusion list)
    samples: list[tuple[str, str, Path, Path]] = []  # (image_rel, label_rel, img_path, label_path)
    skipped = 0
    with open(SRC_CSV, newline="") as f:
        for row in csv.DictReader(f):
            img_rel = row["ImageFile"].strip()
            if img_rel in excluded:
                skipped += 1
                continue
            lbl_rel = row["LabelFile"].strip()
            img_path = (SRC / img_rel).resolve()
            label_path = (SRC / lbl_rel).resolve()
            if not img_path.is_file():
                raise SystemExit(f"missing image: {img_path}")
            if not label_path.is_file():
                raise SystemExit(f"missing label: {label_path}")
            samples.append((img_rel, lbl_rel, img_path, label_path))

    if not samples:
        raise SystemExit("no samples found in the CSV")

    print(f"Excluded by {EXCLUDED_CSV.name}: {skipped}")

    # Parse + validate labels, count objects per class
    parsed: dict[Path, list[list[str]]] = {}
    class_counts: Counter = Counter()
    for _, _, _, label_path in samples:
        lines = parse_label(label_path)
        parsed[label_path] = lines
        for line in lines:
            class_counts[int(line[0])] += 1

    print(f"Samples: {len(samples)}")
    print(f"Objects by class (after remap): {dict(class_counts)}")

    # Deterministic train/val split
    rng = random.Random(SEED)
    indices = list(range(len(samples)))
    rng.shuffle(indices)
    n_val = max(1, round(len(samples) * VAL_FRACTION))
    val_indices = set(indices[:n_val])
    split: dict[str, list] = {"train": [], "val": []}
    for i, sample in enumerate(samples):
        split["val" if i in val_indices else "train"].append(sample)

    # Rebuild the prepared dataset from scratch so stale files never linger.
    if OUT.exists():
        shutil.rmtree(OUT)
    for part in ("train", "val"):
        (OUT / "images" / part).mkdir(parents=True, exist_ok=True)
        (OUT / "labels" / part).mkdir(parents=True, exist_ok=True)

    written = Counter()
    for part, sample_list in split.items():
        for img_rel, _lbl_rel, img_path, label_path in sample_list:
            stem = Path(img_rel).stem  # e.g. 000000
            shutil.copyfile(img_path, OUT / "images" / part / f"{stem}.jpg")
            out_label = OUT / "labels" / part / f"{stem}.txt"
            with open(out_label, "w") as f:
                for line in parsed[label_path]:
                    f.write(" ".join(line) + "\n")
            written[part] += 1

    print(f"Split: train={written['train']} val={written['val']}")

    # Write data.yaml
    data_yaml = {
        "path": str(OUT),
        "train": "images/train",
        "val": "images/val",
        "names": CLASS_NAMES,
    }
    yaml_path = OUT / "data.yaml"
    with open(yaml_path, "w") as f:
        yaml.safe_dump(data_yaml, f, sort_keys=False)
    print(f"Wrote {yaml_path}")
    print("Done. Next: python train_yolo.py")


if __name__ == "__main__":
    main()
