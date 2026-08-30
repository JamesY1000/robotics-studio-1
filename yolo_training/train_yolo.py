<<<<<<< HEAD
#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DEFAULT_DATA = ROOT / "datasets" / "trees" / "data.yaml"
DEFAULT_MODEL = "yolo26n.pt"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Train YOLO on the tree detection database.")
    p.add_argument("--data", default=str(DEFAULT_DATA), help="Path to the data.yaml file")
    p.add_argument("--model", default=DEFAULT_MODEL, help="Pretrained weights or model config to start from")
    p.add_argument("--epochs", type=int, default=100, help="Number of training epochs")
    p.add_argument("--imgsz", type=int, default=640, help="Training image size (px)")
    p.add_argument("--batch", type=int, default=16, help="Batch size (use -1 for auto)")
    p.add_argument("--device", default=None, help="Compute device, e.g. '0' or 'cpu' (None = auto)")
    p.add_argument("--project", default=str(ROOT / "runs"), help="Root directory for experiment outputs")
    p.add_argument("--name", default="trees", help="Experiment name (saved under --project)")
    p.add_argument("--patience", type=int, default=50, help="Early-stopping patience (epochs without improvement)")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    data = Path(args.data)
    if not data.is_file():
        raise SystemExit(f"data.yaml not found: {data}. Run `python prepare_dataset.py` first.")

    model = YOLO(args.model)
    print(f"Model: {args.model}")
    print(model.info())

    model.train(
        data=str(data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        patience=args.patience,
    )
    print(f"Training finished. Best weights: {args.project}/{args.name}/best.pt")

    metrics = model.val(data=str(data), split="val")
    print(f"Validation mAP50-95: {metrics.box.map:.4f}  mAP50: {metrics.box.map50:.4f}")


if __name__ == "__main__":
    main()
=======
from ultralyitics import YOLO

>>>>>>> 8dfe850 (Migrated James' sample generation plugin to Gazebo6 and to using our tree models.)
