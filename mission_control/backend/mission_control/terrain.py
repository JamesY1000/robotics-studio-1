"""Read the actual collision triangles; never infer elevation from image brightness."""
import math
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image

from .models import Calibration, Plan, SpawnPlan, XY, roi_bounds


class Terrain:
    def __init__(self, sim: Path):
        self.sim = sim
        self.preset = json.loads((sim / "config/map_presets.json").read_text())["v2_forest"]
        self.bounds = Calibration(**self.preset["calibration"])
        mesh = sim / self.preset["collision_mesh"]
        data = mesh.read_bytes()
        count, = struct.unpack_from("<I", data, 80)
        # Spatial index, with the same triangles used by Gazebo collisions.
        self.cells: dict[tuple[int, int], list] = {}
        for i in range(count):
            values = struct.unpack_from("<12fH", data, 84 + 50 * i)
            a, b, c = values[3:6], values[6:9], values[9:12]
            for x in range(math.floor(min(a[0], b[0], c[0])), math.floor(max(a[0], b[0], c[0])) + 1):
                for y in range(math.floor(min(a[1], b[1], c[1])), math.floor(max(a[1], b[1], c[1])) + 1):
                    self.cells.setdefault((x, y), []).append((a, b, c))
        self.water = Image.open(sim / self.preset["water_mask"]).convert("L")
        world = ET.parse(sim / "worlds/v2_forest.sdf").getroot().find("world")
        self.obstacles = []
        self.fixed_stations = []
        for item in world.findall("include"):
            uri = item.findtext("uri", "")
            if uri == "model://ground_station":
                pose = [float(v) for v in item.findtext("pose", "0 0 0 0 0 0").split()]
                self.fixed_stations.append((pose[0], pose[1]))
            if any(part in uri for part in ("tree", "rock", "bush")):
                pose = [float(v) for v in item.findtext("pose", "0 0 0 0 0 0").split()]
                self.obstacles.append((pose[0], pose[1]))

    def height(self, x: float, y: float) -> float:
        for a, b, c in self.cells.get((math.floor(x), math.floor(y)), []):
            den = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
            if abs(den) < 1e-12:
                continue
            u = ((b[1]-c[1])*(x-c[0]) + (c[0]-b[0])*(y-c[1])) / den
            v = ((c[1]-a[1])*(x-c[0]) + (a[0]-c[0])*(y-c[1])) / den
            if u >= -1e-6 and v >= -1e-6 and u + v <= 1 + 1e-6:
                return u*a[2] + v*b[2] + (1-u-v)*c[2]
        raise ValueError("No terrain collision triangle at selected coordinates")

    def check_bounds(self, name: str, point: XY, radius: float):
        if not self.bounds.contains(point, radius):
            raise ValueError(f"{name} must be inside the terrain with {radius:g} m edge clearance")

    def check_point(self, name: str, point: XY, radius: float):
        if not self.bounds.contains(point, radius):
            raise ValueError(f"{name} must be inside the terrain with {radius:g} m edge clearance")
        for dx, dy in [(0, 0), (-radius, 0), (radius, 0), (0, -radius), (0, radius)]:
            u, v = self.bounds.to_image(XY(x=point.x+dx, y=point.y+dy))
            pixel = (round(u*(self.water.width-1)), round(v*(self.water.height-1)))
            if self.water.getpixel(pixel) > 127:
                raise ValueError(f"{name} overlaps the water mask; choose dry terrain")
        if any(math.hypot(point.x-x, point.y-y) < radius+0.8 for x, y in self.obstacles):
            raise ValueError(f"{name} is too close to a tree, bush or rock origin; choose a clearing")

    def placements(self, plan: Plan) -> dict:
        station = XY(x=plan.start.x+plan.station_offset.x, y=plan.start.y+plan.station_offset.y)
        self.check_point("Robot start", plan.start, 1)
        self.check_point("Ground station", station, 2)
        self.check_point("ROI", plan.roi, 0.5)
        roi_min, roi_max = roi_bounds(plan.roi, plan.roi_second)
        for corner in (roi_min, roi_max):
            self.check_bounds("ROI box", corner, 0.5)
        if math.hypot(plan.station_offset.x, plan.station_offset.y) < 4:
            raise ValueError("Ground station must be at least 4 m from the robot start")
        # Wheel lower extent: radius 0.1651 - joint Z 0.03282 = 0.13228 m.
        # Sample footprint to avoid initial penetration on sloping terrain.
        ground = self.height(plan.start.x, plan.start.y)
        footprint = max(self.height(plan.start.x+dx, plan.start.y+dy)
                        for dx in (-0.6, 0, 0.6) for dy in (-0.45, 0, 0.45))
        station_z = max(self.height(station.x+dx, station.y+dy)
                        for dx in (-2, 0, 2) for dy in (-2, 0, 2))
        return {
            "husky": {**plan.start.model_dump(), "z": footprint + 0.13228 + 0.10, "yaw": 0.0},
            # Downward sensor is 0.1 m below the base frame.
            "parrot": {**plan.start.model_dump(), "z": ground + plan.parrot_clearance + 0.1, "yaw": 0.0},
            "station": {**station.model_dump(), "z": station_z + 0.05, "yaw": 0.0},
            "roi": {**plan.roi.model_dump(), "z": self.height(plan.roi.x, plan.roi.y)},
            "roi_box": {"x_min": roi_min.x, "x_max": roi_max.x,
                        "y_min": roi_min.y, "y_max": roi_max.y,
                        "z": self.height(plan.roi.x, plan.roi.y)},
        }

    def spawn_placements(self, plan: SpawnPlan) -> dict:
        self.check_point("Robot start", plan.start, 1)
        if any(math.hypot(plan.start.x-x, plan.start.y-y) < 4 for x, y in self.fixed_stations):
            raise ValueError("Robot start must be at least 4 m from the world's ground station")
        ground = self.height(plan.start.x, plan.start.y)
        footprint = max(self.height(plan.start.x+dx, plan.start.y+dy)
                        for dx in (-0.6, 0, 0.6) for dy in (-0.45, 0, 0.45))
        return {
            "husky": {**plan.start.model_dump(), "z": footprint + 0.13228 + 0.10, "yaw": 0.0},
            "parrot": {**plan.start.model_dump(), "z": ground + plan.parrot_clearance + 0.1, "yaw": 0.0},
        }

    def write_world(self, destination: Path, placements: dict):
        tree = ET.parse(self.sim / "worlds/v2_forest.sdf")
        world = tree.getroot().find("world")
        stations = [item for item in world.findall("include")
                    if item.findtext("uri") == "model://ground_station"]
        if len(stations) != 1:
            raise ValueError("Expected exactly one ground station in source world")
        p = placements["station"]
        stations[0].find("pose").text = f"{p['x']} {p['y']} {p['z']} 0 0 0"
        tree.write(destination, encoding="utf-8", xml_declaration=True)
