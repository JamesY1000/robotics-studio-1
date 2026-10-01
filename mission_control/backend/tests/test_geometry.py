import math
from types import SimpleNamespace

import pytest
from mission_control.models import Calibration, XY
from rs1_alignment.geometry import compose, registration, wrap, yaw


@pytest.mark.parametrize("uv,world", [((0, 0), (-100, -100)), ((1, 0), (100, -100)),
    ((0, 1), (-100, 100)), ((1, 1), (100, 100)), ((.5, .5), (0, 0)), ((.73, .16), (46, -68))])
def test_edge_conversion(uv, world):
    c = Calibration(x_min=-100, x_max=100, y_min=-100, y_max=100)
    p = c.to_world(*uv)
    assert (p.x, p.y) == pytest.approx(world)
    assert c.to_image(p) == pytest.approx(uv)


@pytest.mark.parametrize("world,local", [((20, -35, 0), (4, -9, 1.2)),
    ((-10, 70, -2.8), (-30, 15, 2.1)), ((90, -48, .1), (90, -48, -.7))])
def test_registration_nonzero_local_pose(world, local):
    transform = registration(world, local)
    assert compose(transform, local) == pytest.approx(world)
    # A second translated pose must retain its rotated displacement.
    next_pose = compose(transform, (local[0]+1, local[1], local[2]))
    assert next_pose[:2] == pytest.approx((world[0]+math.cos(transform[2]), world[1]+math.sin(transform[2])))


def test_model_to_base_offset():
    assert compose((10, 20, math.pi/2), (2, 0, -.2)) == pytest.approx((10, 22, math.pi/2-.2))
    q = SimpleNamespace(x=0, y=0, z=math.sin(.6), w=math.cos(.6))
    assert yaw(q) == pytest.approx(1.2)
    assert wrap(math.pi+0.1) == pytest.approx(-math.pi+0.1)


def test_terrain_collision_and_spawn(terrain, plan, tmp_path):
    import xml.etree.ElementTree as ET
    assert terrain.height(-100, -100) == pytest.approx(2.875156, abs=1e-5)
    assert terrain.height(100, 100) == pytest.approx(3.961117, abs=1e-5)
    p = terrain.placements(plan)
    assert p["husky"]["x"] == p["parrot"]["x"] == plan.start.x
    assert p["husky"]["yaw"] == p["parrot"]["yaw"] == 0
    assert p["parrot"]["z"] > p["husky"]["z"] + 2
    assert p["parrot"]["z"] == pytest.approx(terrain.height(plan.start.x, plan.start.y)+4.1)
    terrain.write_world(tmp_path / "world.sdf", p)
    includes = ET.parse(tmp_path / "world.sdf").findall(".//world/include")
    stations = [i for i in includes if i.findtext("uri") == "model://ground_station"]
    assert len(stations) == 1
    assert [float(v) for v in stations[0].findtext("pose").split()][:3] == pytest.approx([p["station"][k] for k in ("x", "y", "z")])


def test_invalid_placements(terrain, plan):
    from pydantic import ValidationError
    with pytest.raises(ValidationError):
        XY(x=float('nan'), y=0)
    with pytest.raises(ValueError, match="edge clearance"):
        terrain.placements(plan.model_copy(update={"start": XY(x=100, y=100)}))
    with pytest.raises(ValueError, match="at least 4"):
        terrain.placements(plan.model_copy(update={"station_offset": XY(x=0, y=0)}))


def test_map_assets(terrain):
    from PIL import Image
    import json
    import xml.etree.ElementTree as ET
    image = terrain.sim / "models/forest_plane/materials/textures/forest_overhead.png"
    assert Image.open(image).size == (1024, 1024)
    metadata = json.loads((image.parent / "source/terrain_metadata.json").read_text())
    assert metadata["physical_dimensions_m"] == {"width": 200, "height": 200}
    world = ET.parse(terrain.sim / "worlds/v2_forest.sdf")
    forest = [i for i in world.findall(".//world/include") if i.findtext("name") == "forest_plane"][0]
    assert all(float(v) == 0 for v in forest.findtext("pose").split())
