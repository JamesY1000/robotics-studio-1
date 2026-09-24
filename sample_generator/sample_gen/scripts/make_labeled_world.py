#!/usr/bin/env python3
"""Generate ``v2_forest_labeled.sdf`` from ``v2_forest.sdf``.

Every tree ``<include>`` in the rs1_ws forest world is re-pointed at the
``*_labeled`` model variant (which carries the ObjectLabelPlugin that publishes
ground-truth boxes + TF for the sample generator). The world ``<name>`` is left
as ``v2_forest`` so the existing ``world/v2_forest/{create,set_pose,remove}``
service paths and the sample generator's ``world_name`` config are unchanged.
"""

from pathlib import Path

# <repo-root>/sample_generator/sample_gen/scripts/ -> repo root is 3 parents up.
REPO_ROOT = Path(__file__).resolve().parents[3]

SRC = REPO_ROOT / "rs1_ws" / "src" / "rs1_sim" / "worlds" / "v2_forest.sdf"
DST = REPO_ROOT / "rs1_ws" / "src" / "rs1_sim" / "worlds" / "v2_forest_labeled.sdf"

REPLACEMENTS = {
    "model://trees/oak_tree</uri>": "model://trees/oak_tree_labeled</uri>",
    "model://trees/pine_tree</uri>": "model://trees/pine_tree_labeled</uri>",
}


def main() -> None:
    text = SRC.read_text()
    for old, new in REPLACEMENTS.items():
        text = text.replace(old, new)
    DST.write_text(text)
    print(f"Wrote {DST} ({len(text)} chars)")


if __name__ == "__main__":
    main()
