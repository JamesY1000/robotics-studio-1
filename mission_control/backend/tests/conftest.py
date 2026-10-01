import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "rs1_ws/src/rs1_alignment"))


@pytest.fixture(scope="session")
def terrain():
    from mission_control.terrain import Terrain
    return Terrain(ROOT / "rs1_ws/src/rs1_sim")


@pytest.fixture
def plan(terrain):
    from uuid import uuid4
    from mission_control.models import Plan, XY
    for x, y in [(90, -48), (88, -48), (85, -45), (80, -45), (70, -40), (0, 0)]:
        p = Plan(session_id=uuid4(), start=XY(x=x, y=y), roi=XY(x=x-2, y=y), station_offset=XY(x=0, y=5))
        try:
            terrain.placements(p)
            return p
        except ValueError:
            pass
    pytest.fail("No dry clearing found for fixture")
