"""ROS transport only. Simulation process ownership lives in SessionManager."""

import asyncio
import copy
import json
import math
import threading
import time

from .telemetry_config import TELEMETRY_NODE
from .safety import reset_ready


class RequestUncertain(RuntimeError):
    """A dispatched command may have run even though no response arrived."""


class UnavailableAdapter:
    def __init__(self, reason):
        self.reason = reason

    def snapshot(self):
        return {"available": False, "error": self.reason, "robots": {}, "checks": {}}

    def close(self):
        pass


class RosAdapter:
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.action import ActionClient
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from tf2_ros import Buffer, TransformListener
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from lifecycle_msgs.srv import GetState
        from rs1_interfaces.msg import MissionStatus, SafetyState
        from rs1_interfaces.srv import SafetyStop
        from rs1_interfaces.msg import TreeMap, BiomassSummary, TreeSamplingTarget
        from rs1_interfaces.action import TraverseToROI, ExploreToPose
        from nav2_msgs.action import (
            NavigateToPose,
            NavigateThroughPoses,
            ComputePathToPose,
        )
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Joy

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node(
            "mission_control_backend",
            context=self.context,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.buffer = Buffer(node=self.node)
        self.listener = TransformListener(self.buffer, self.node)
        self.lock = threading.Lock()
        self.data = {
            "available": True,
            "robots": {},
            "alignment": {},
            "mission": {},
            "checks": {},
            "trees": [],
            "total_agb_kg": None,
            "biomass": None,
        }
        self.biomass_summaries = {}
        self.tree_map_key = None
        self.sampling_target = None
        self.worlds = {}
        self.last_clock = 0
        self.clock_changed_at = 0
        self.safety_type = SafetyStop
        self.safety_client = self.node.create_client(SafetyStop, "/safety/stop")
        self.trigger_type = Trigger
        self.start_client = self.node.create_client(Trigger, "/mission/start")
        self.cancel_client = self.node.create_client(Trigger, "/mission/cancel")
        self.command_clients = {
            "start": self.start_client,
            "cancel": self.cancel_client,
        }
        for operation in (
            "manual_husky",
            "manual_parrot",
            "resume",
            "input_keyboard",
            "input_controller",
        ):
            self.command_clients[operation] = self.node.create_client(
                Trigger, f"/mission/{operation}"
            )
        self.twist_type = Twist
        self.manual_publishers = {
            robot: self.node.create_publisher(Twist, f"/{robot}/cmd_vel_manual", 1)
            for robot in ("husky1", "parrot1")
        }
        self.parameters = self.node.create_client(
            SetParametersAtomically,
            "/mission_coordinator_node/set_parameters_atomically",
        )
        self.actions = {
            name: ActionClient(self.node, kind, name)
            for name, kind in [
                ("/husky1/traverse_to_roi", TraverseToROI),
                ("/parrot1/traverse_to_roi", TraverseToROI),
                ("/husky1/navigate_through_poses", NavigateThroughPoses),
                ("/parrot1/navigate_to_pose", NavigateToPose),
                ("/parrot1/compute_path_to_pose", ComputePathToPose),
                ("/parrot1/explore_to_pose", ExploreToPose),
            ]
        }
        self.lifecycle = {
            f"/{r}/{n}": self.node.create_client(GetState, f"/{r}/{n}/get_state")
            for r in ("husky1", "parrot1")
            for n in ("controller_server", "planner_server", "bt_navigator")
        }
        self.lifecycle_values = {}
        self.lifecycle_pending = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            String, "/mission/alignment", self.on_alignment, latched
        )
        self.node.create_subscription(
            String, "/simulation/session", self.on_world, latched
        )
        self.node.create_subscription(SafetyState, "/safety/state", self.on_safety, latched)
        self.node.create_subscription(
            MissionStatus, "/mission/status", self.on_mission, latched
        )
        self.node.create_subscription(
            TreeMap, "/mission/tree_map", self.on_tree_map, latched
        )
        self.node.create_subscription(
            TreeSamplingTarget, "/husky1/tree_sampling_target", self.on_sampling_target, latched
        )
        self.node.create_subscription(
            BiomassSummary, "/carbon/summary", self.on_biomass, latched
        )
        self.node.create_subscription(
            Joy, "/teleop/joy", self.on_joy, rclpy.qos.qos_profile_sensor_data
        )
        self.node.create_timer(0.2, self.poll)
        self.node.create_timer(1.0, self.poll_lifecycle, clock=rclpy.clock.Clock())
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    @staticmethod
    def map_key(header):
        return header.frame_id, header.stamp.sec, header.stamp.nanosec

    def apply_biomass(self):
        # Called with the data lock held, after either side of the latched handshake arrives
        mission_id = self.data["mission"].get("instance_id")
        summary = self.biomass_summaries.get((mission_id, self.tree_map_key))
        if self.tree_map_key is None:
            summary = next((value for key, value in reversed(self.biomass_summaries.items())
                            if key[0] == mission_id), None)
        self.data["biomass"] = {k: v for k, v in summary.items() if k != "map_key"} if summary else None
        self.data["total_agb_kg"] = summary["total_agb_kg"] if summary else None

    def on_biomass(self, msg):
        if not msg.mission_id or not msg.header.frame_id:
            return
        valid = (msg.valid and math.isfinite(msg.total_agb_kg) and msg.total_agb_kg >= 0
                 and 0 <= msg.estimated_count <= msg.measured_count <= msg.tree_count)
        summary = {"map_key": self.map_key(msg.header), "valid": valid,
                   "total_agb_kg": msg.total_agb_kg if valid and msg.estimated_count else None,
                   "tree_count": msg.tree_count, "measured_count": msg.measured_count,
                   "estimated_count": msg.estimated_count, "message": msg.message}
        with self.lock:
            self.biomass_summaries[(msg.mission_id, summary["map_key"])] = summary
            # Keep a small cache for status/map messages that arrive in a different order
            while len(self.biomass_summaries) > 4:
                del self.biomass_summaries[next(iter(self.biomass_summaries))]
            self.apply_biomass()

    def on_world(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict) or not isinstance(
                value.get("instance_id"), str
            ):
                return
        except ValueError:
            return
        with self.lock:
            self.worlds[value["instance_id"]] = (value, time.monotonic())

    def on_alignment(self, msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.data["alignment"] = value
            self.data["alignment_at"] = time.monotonic()

    def on_safety(self, msg):
        with self.lock:
            self.data["safety"] = {key: getattr(msg, key) for key in (
                "instance_id", "revision", "husky_stopped", "parrot_stopped")}
            self.data["safety_at"] = time.monotonic()

    def on_mission(self, msg):
        with self.lock:
            previous = self.data["mission"]
            if previous and previous.get("instance_id") != msg.instance_id:
                self.tree_map_key = None
            self.data["mission"] = {
                key: getattr(msg, key)
                for key in (
                    "session_id",
                    "instance_id",
                    "state",
                    "parrot_stage",
                    "husky_stage",
                    "message",
                    "cleanup_complete",
                    "returning_home",
                    "control_mode",
                    "manual_input",
                    "input_revision",
                    "safety_instance_id",
                    "safety_revision",
                )
            }
            # Invalidate old keyboard connections even if the mode changes back between dashboard polls
            changed = any(
                previous.get(key) != self.data["mission"][key]
                for key in (
                    "instance_id",
                    "state",
                    "control_mode",
                    "manual_input",
                    "input_revision",
                    "safety_instance_id",
                    "safety_revision",
                )
            )
            self.data["mission"]["control_revision"] = previous.get(
                "control_revision", 0
            ) + int(changed)
            self.data["mission"]["roi"] = {
                "frame_id": msg.roi.header.frame_id,
                "x": msg.roi.pose.position.x,
                "y": msg.roi.pose.position.y,
                "z": msg.roi.pose.position.z,
                "min": {"x": msg.roi_min.x, "y": msg.roi_min.y},
                "max": {"x": msg.roi_max.x, "y": msg.roi_max.y},
                "progress": msg.roi_progress,
            }
            self.data["mission_at"] = time.monotonic()
            self.apply_biomass()
            self.apply_sampling_target()

    def on_joy(self, msg):
        # Presence is only a dashboard hint - the selectors enforce their own input timeout
        with self.lock:
            self.data["controller_at"] = (
                time.monotonic() if len(msg.buttons) > 9 and len(msg.axes) >= 4 else 0
            )

    def on_tree_map(self, msg):
        # Latched map of detected/located trees published by the coordinator from
        # the parrot's SampleROI action result. Tree positions are in `world`.
        with self.lock:
            self.tree_map_key = self.map_key(msg.header)
            self.data["trees"] = [
                {
                    "x": tree.location.x,
                    "y": tree.location.y,
                    "z": tree.location.z,
                    "type": tree.tree_type,
                    "observations": tree.observations,
                }
                for tree in msg.trees
            ]
            self.apply_biomass()
            self.apply_sampling_target()

    def on_sampling_target(self, msg):
        with self.lock:
            self.sampling_target = msg
            self.apply_sampling_target()

    def apply_sampling_target(self):
        # Match the mission and map so a retained target cannot leak into a new run
        msg = getattr(self, "sampling_target", None)
        mission = self.data.get("mission", {})
        self.data["sampling_target"] = None
        if (msg is None or not msg.active or msg.mission_id != mission.get("instance_id")
                or self.map_key(msg.header) != self.tree_map_key
                or mission.get("state") not in ("sampling", "manual_assistance")
                or msg.tree_index >= len(self.data.get("trees", []))):
            return
        x, y = msg.centre.x, msg.centre.y
        if not all(math.isfinite(v) for v in (x, y, msg.min_range_m, msg.max_range_m)):
            return
        if not 0 < msg.min_range_m < msg.max_range_m:
            return
        if msg.header.frame_id != "world":
            from rclpy.time import Time
            try:
                tf = self.buffer.lookup_transform("world", msg.header.frame_id, Time()).transform
                q = tf.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                x, y = (math.cos(yaw)*x-math.sin(yaw)*y+tf.translation.x,
                        math.sin(yaw)*x+math.cos(yaw)*y+tf.translation.y)
            except Exception:
                return
        self.data["sampling_target"] = dict(
            tree_index=msg.tree_index, x=x, y=y, refined=msg.refined,
            min_range_m=msg.min_range_m, max_range_m=msg.max_range_m)

    def poll_lifecycle(self):
        from lifecycle_msgs.srv import GetState

        for name, client in self.lifecycle.items():
            pending = self.lifecycle_pending.get(name)
            if pending:
                future, sent = pending
                if future.done():
                    try:
                        self.lifecycle_values[name] = (
                            future.result().current_state.id == 3,
                            time.monotonic(),
                        )
                    except Exception:
                        self.lifecycle_values[name] = (False, time.monotonic())
                    del self.lifecycle_pending[name]
                elif time.monotonic() - sent > 3:
                    future.cancel()
                    del self.lifecycle_pending[name]
            elif client.service_is_ready():
                self.lifecycle_pending[name] = (
                    client.call_async(GetState.Request()),
                    time.monotonic(),
                )

    def poll(self):
        from rclpy.time import Time
        from tf2_ros import TransformException

        now = self.node.get_clock().now().nanoseconds
        wall = time.monotonic()
        if now != self.last_clock:
            self.last_clock = now
            self.clock_changed_at = wall
        robots = {}
        for robot in ("husky1", "parrot1"):
            try:
                t = self.buffer.lookup_transform("world", robot + "_base_link", Time())
                stamp = Time.from_msg(t.header.stamp).nanoseconds
                q, p = t.transform.rotation, t.transform.translation
                heading = math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                )
                robots[robot] = {
                    "x": p.x,
                    "y": p.y,
                    "yaw": heading,
                    "stamp": stamp / 1e9,
                    "fresh": -0.1 <= (now - stamp) / 1e9 < 2,
                }
            except TransformException:
                robots[robot] = {"fresh": False}
        checks = {
            "mission_services": self.start_client.service_is_ready()
            and self.cancel_client.service_is_ready()
        }
        checks.update(
            {name: client.server_is_ready() for name, client in self.actions.items()}
        )
        checks.update(
            {
                name: self.lifecycle_values.get(name, (False, 0))[0]
                and wall - self.lifecycle_values.get(name, (False, 0))[1] < 4
                for name in self.lifecycle
            }
        )
        with self.lock:
            self.data.update(robots=robots, checks=checks, sim_time=now / 1e9)

    def snapshot(self):
        with self.lock:
            value = copy.deepcopy(self.data)
            current = [
                entry for entry, at in self.worlds.values() if time.monotonic() - at < 3
            ]
            value["world"] = (
                {
                    **copy.deepcopy(current[-1]),
                    "fresh": True,
                    "conflict": len(current) > 1,
                }
                if current
                else {"fresh": False}
            )
        wall = time.monotonic()
        value["controller"] = {"connected": wall - value.get("controller_at", 0) < 0.5}
        value["checks"]["clock"] = wall - self.clock_changed_at < 2
        for robot in value["robots"].values():
            robot["fresh"] = robot["fresh"] and value["checks"]["clock"]
        value["checks"]["mission_status"] = wall - value.get("mission_at", 0) < 3
        value["checks"]["alignment_status"] = wall - value.get("alignment_at", 0) < 3
        safety = value.setdefault("safety", {})
        safety["fresh"] = bool(safety.get("instance_id")) and wall - value.get("safety_at", 0) < 0.5
        safety["service_ready"] = self.safety_client.service_is_ready()
        safety["reset_ready"] = reset_ready(value)
        return value

    async def set_safety_stop(self, request):
        if not self.safety_client.service_is_ready():
            raise RuntimeError("Software stop service is unavailable; stop state is not confirmed")
        # Forward the UI's observed revision unchanged so a delayed Reset cannot clear a newer Stop
        future = self.safety_client.call_async(self.safety_type.Request(**request.model_dump()))
        deadline = time.monotonic() + 3
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                raise RequestUncertain("Software stop request timed out; its outcome is unknown, check the live stop state")
            await asyncio.sleep(0.02)
        try:
            result = future.result()
        except Exception as exc:
            raise RequestUncertain(f"Software stop response failed; outcome unknown: {exc}") from exc
        if not result.success:
            raise RuntimeError(result.message)
        return {"message": result.message}

    async def call(self, kind):
        client = self.command_clients[kind]
        if not client.service_is_ready():
            raise RuntimeError(f"/mission/{kind} is unavailable")
        future = client.call_async(self.trigger_type.Request())
        deadline = time.monotonic() + 10
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                raise RequestUncertain(
                    f"/mission/{kind} timed out; inspect the coordinator before retrying"
                )
            await asyncio.sleep(0.05)
        try:
            result = future.result()
        except Exception as exc:
            raise RequestUncertain(f"/mission/{kind} response failed: {exc}") from exc
        if not result.success:
            raise RuntimeError(result.message)
        return result.message

    def publish_manual(self, robot, command=None):
        msg = self.twist_type()
        if command is not None and command.enabled:
            # Convert keyboard directions to velocities - the selector also enforces its own limits
            msg.linear.x = command.x * 0.5
            msg.angular.z = command.yaw * 0.7
            if robot == "parrot1":
                msg.linear.y = command.y * 0.5
                msg.linear.z = command.z * 0.3
        self.manual_publishers[robot].publish(msg)

    async def configure_roi(self, roi, roi_box=None):
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import SetParametersAtomically

        if not self.parameters.service_is_ready():
            raise RuntimeError("Mission coordinator parameter services are unavailable")
        values = [
            Parameter("roi_frame", value="world"),
            *[
                Parameter(f"roi_{axis}", value=float(roi[axis]))
                for axis in ("x", "y", "z")
            ],
        ]
        if roi_box is not None:
            values.extend(
                [
                    Parameter("roi_x_min", value=float(roi_box["x_min"])),
                    Parameter("roi_x_max", value=float(roi_box["x_max"])),
                    Parameter("roi_y_min", value=float(roi_box["y_min"])),
                    Parameter("roi_y_max", value=float(roi_box["y_max"])),
                ]
            )
        future = self.parameters.call_async(
            SetParametersAtomically.Request(
                parameters=[value.to_parameter_msg() for value in values]
            )
        )
        deadline = time.monotonic() + 10
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                raise RuntimeError(
                    "ROI update timed out. Check the coordinator before retrying."
                )
            await asyncio.sleep(0.05)
        result = future.result().result
        if not result.successful:
            raise RuntimeError(result.reason)

    def telemetry_available(self):
        return any(
            f"{namespace.rstrip('/')}/{name}" == TELEMETRY_NODE
            for name, namespace in self.node.get_node_names_and_namespaces()
        )

    def foreign_nodes(self):
        return [
            f"{namespace.rstrip('/')}/{name}"
            for name, namespace in self.node.get_node_names_and_namespaces()
            if name not in ("mission_control_backend", "_ros2cli_daemon")
            and not name.startswith("_ros2cli")
        ]

    def close(self):
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.node.destroy_node()
        self.context.shutdown()


def create_adapter():
    try:
        return RosAdapter()
    except ImportError as exc:
        return UnavailableAdapter(
            f"ROS adapter unavailable: {exc}. Source ROS Humble and rs1_ws/install/setup.bash, then restart the backend."
        )
