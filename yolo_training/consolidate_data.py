#!/usr/bin/env python3
"""Consolidate the tree-detection samples from ``to_check/`` into ``database/``.

The raw sample sets under ``to_check`` reuse the same image numbers
(``000000``, ``000001``, ...), so they cannot be merged as-is. This script
builds a single, uniquely-numbered database under
``database/camera_samples/``:

    * ``to_check/in_env``  -> ALL samples, keeping their numbers ``000000..``
    * ``to_check/out_env`` -> single-tree-type samples only (oak-only or
      pine-only), numbered after every in_env sample
    * ``to_check/out_env`` -> mixed oak+pine samples are skipped entirely

Both the raw images and the boxed ``labelled/`` previews are copied, keeping the
original 1-based class ids (1 = OAK_TREE, 2 = PINE_TREE);
``prepare_dataset.py`` performs the 1-based -> 0-based remap.

``database/excluded.csv`` holds one relative image path per row (e.g.
``images/000010.jpg``). ``prepare_dataset.py`` skips every image listed there, so
after a manual review you can add or remove rows to control what is used.
Exclusions that still refer to images in the database are preserved across
re-runs of this script.

Usage:
    python consolidate_data.py
"""

from __future__ import annotations

import csv
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent
TO_CHECK = ROOT / "to_check"
IN_ENV = TO_CHECK / "in_env" / "camera_samples"
OUT_ENV = TO_CHECK / "out_env" / "camera_samples"

OUT_CAM = ROOT / "database" / "camera_samples"
OUT_CSV = OUT_CAM / "sample_database.csv"
EXCLUDED_CSV = ROOT / "database" / "excluded.csv"

IMG_SUBDIR = "images"
LBL_SUBDIR = "labels"
LABELLED_SUBDIR = "labelled"


def _read_rows(cam_dir: Path) -> list[dict[str, str]]:
    """Read a source ``sample_database.csv``, sorted by numeric id."""
    csv_path = cam_dir / "sample_database.csv"
    if not csv_path.is_file():
        raise SystemExit(f"sample database not found: {csv_path}")
    with open(csv_path, newline="") as f:
        rows = [row for row in csv.DictReader(f) if (row.get("ImageFile") or "").strip()]
    rows.sort(key=lambda row: int(row["ID"]))
    return rows


def _label_classes(label_path: Path) -> set[int]:
    """Return the distinct (1-based) class ids present in a YOLO label file."""
    classes: set[int] = set()
    with open(label_path) as f:
        for line in f:
            line = line.strip()
            if line:
                classes.add(int(line.split()[0]))
    return classes


def _count_objects(label_path: Path) -> int:
    """Count the number of object rows in a YOLO label file."""
    with open(label_path) as f:
        return sum(1 for line in f if line.strip())


def _load_exclusions(path: Path) -> list[str]:
    """Read existing ``excluded.csv`` rows (accepts a header or plain lines)."""
    if not path.is_file():
        return []
    rows: list[str] = []
    with open(path, newline="") as f:
        for line in f:
            value = line.split(",")[0].strip()
            if value and value.lower() != "imagefile":
                rows.append(value)
    return rows


def _find_labelled(cam_dir: Path, stem: str) -> Path | None:
    """Find the boxed ``labelled/`` preview for a given image stem, if present."""
    labelled_dir = cam_dir / LABELLED_SUBDIR
    return next(iter(sorted(labelled_dir.glob(f"{stem}.*"))), None)


def _copy_sample(src_cam: Path, img_rel: str, lbl_rel: str, new_id: int) -> int:
    """Copy one sample (image, label, preview) under ``new_id``.

    The label contents are copied verbatim; returns the object count.
    """
    src_img = src_cam / img_rel
    src_lbl = src_cam / lbl_rel
    dst_stem = f"{new_id:06d}"

    shutil.copyfile(src_img, OUT_CAM / IMG_SUBDIR / f"{dst_stem}{src_img.suffix}")
    shutil.copyfile(src_lbl, OUT_CAM / LBL_SUBDIR / f"{dst_stem}{src_lbl.suffix}")

    labelled = _find_labelled(src_cam, Path(img_rel).stem)
    if labelled is not None:
        shutil.copyfile(labelled, OUT_CAM / LABELLED_SUBDIR / f"{dst_stem}{labelled.suffix}")

    return _count_objects(src_lbl)


def main() -> None:
    # Start from a clean output so re-runs never leave stale files behind.
    if OUT_CAM.exists():
        shutil.rmtree(OUT_CAM)
    for sub in (IMG_SUBDIR, LBL_SUBDIR, LABELLED_SUBDIR):
        (OUT_CAM / sub).mkdir(parents=True, exist_ok=True)
    EXCLUDED_CSV.parent.mkdir(parents=True, exist_ok=True)

    records: list[tuple[int, str, str, int]] = []

    # 1) in_env: keep everything, preserving the original numbering.
    in_rows = _read_rows(IN_ENV)
    max_in_id = -1
    for row in in_rows:
        new_id = int(row["ID"])
        num = _copy_sample(IN_ENV, row["ImageFile"].strip(), row["LabelFile"].strip(), new_id)
        records.append((new_id, f"{IMG_SUBDIR}/{new_id:06d}.jpg", f"{LBL_SUBDIR}/{new_id:06d}.txt", num))
        max_in_id = max(max_in_id, new_id)

    next_id = max_in_id + 1

    # 2) out_env: keep single-tree-type samples only; skip mixed oak+pine ones.
    out_rows = _read_rows(OUT_ENV)
    single: list[dict[str, str]] = []
    mixed: list[dict[str, str]] = []
    for row in out_rows:
        classes = _label_classes(OUT_ENV / row["LabelFile"].strip())
        (single if len(classes) == 1 else mixed).append(row)

    for row in single:
        num = _copy_sample(OUT_ENV, row["ImageFile"].strip(), row["LabelFile"].strip(), next_id)
        records.append((next_id, f"{IMG_SUBDIR}/{next_id:06d}.jpg", f"{LBL_SUBDIR}/{next_id:06d}.txt", num))
        next_id += 1

    # Write the consolidated inventory (generator-compatible schema).
    with open(OUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ID", "ImageFile", "LabelFile", "NumObjects"])
        writer.writerows(records)

    # Write the exclusion list consumed by prepare_dataset.py, preserving any
    # manual review exclusions that still refer to images in the database.
    valid_images = {img_rel for _, img_rel, _, _ in records}
    preserved = [rel for rel in _load_exclusions(EXCLUDED_CSV) if rel in valid_images]
    with open(EXCLUDED_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ImageFile"])
        for img_rel in preserved:
            writer.writerow([img_rel])

    print(f"in_env copied (all):        {len(in_rows)} (ids 000000..{max_in_id:06d})")
    print(f"out_env single-tree type:   {len(single)} (included)")
    print(f"out_env mixed:              {len(mixed)} (skipped, not copied)")
    print(f"total in database:          {len(records)}")
    print(f"exclusions preserved:       {len(preserved)}")
    print(f"Wrote {OUT_CSV}")
    print(f"Wrote {EXCLUDED_CSV}")
    print("Done. Next: python prepare_dataset.py")


if __name__ == "__main__":
    main()
