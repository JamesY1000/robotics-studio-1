"""Dashboard connection to ROS processes owned by the operator's terminals."""
import copy
import time

from .models import ExternalStart
from .ros_adapter import RequestUncertain
from .session import Conflict, SessionManager

ROBOTS = ("husky1", "parrot1")
TERMINAL = ("complete", "cancelled", "failed")


class ExternalSessionManager(SessionManager):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs, restore=False)
        self.state.update(phase="waiting", message="Waiting for the separately launched ROS stack.")
        self.command = None
        self.command_error = None
        self.alignment_instance = None
        self.mission_instance = None
        self.last_sim_time = None

    def persist(self):
        # ROS is authoritative. Never restore a locally saved mission over it.
        pass

    def missing(self, ros):
        missing = [name for name, ready in ros.get("checks", {}).items() if not ready]
        if not ros.get("available"):
            missing.append("ROS adapter")
        if not ros.get("checks"):
            missing.append("ROS interfaces")
        alignment = ros.get("alignment", {})
        if self.alignment_instance and alignment.get("instance_id") != self.alignment_instance:
            missing.append("TF refresh for restarted simulation")
        if self.last_sim_time is not None and ros.get("sim_time", self.last_sim_time) < self.last_sim_time:
            missing.append("TF refresh after clock reset")
        if alignment.get("world_name") != "v2_forest":
            missing.append("v2_forest world alignment")
        if alignment.get("invalid"):
            missing.append("new simulation alignment after clock reset")
        for robot in ROBOTS:
            # Initial verification belongs to the simulation's registration lifetime.
            # A late dashboard must not demand that accumulated localization drift
            # look like a new registration. Fresh TF/status and invalidation still gate.
            if not (alignment.get("initial_verification", {}).get(robot) or alignment.get("ready", {}).get(robot)):
                missing.append(f"{robot} alignment")
            if not ros.get("robots", {}).get(robot, {}).get("fresh"):
                missing.append(f"{robot} localization")
        return missing

    def view(self, ros):
        mission = ros.get("mission", {})
        connected = ros.get("checks", {}).get("mission_status", False)
        phase = mission.get("state", "waiting") if connected else "waiting"
        phase = "idle" if phase == "ready" else phase
        message = mission.get("message") or "Connected to the separately launched mission coordinator."
        missing = self.missing(ros)
        can_start = connected and phase == "idle" and not missing and self.command is None
        if not connected:
            message = "Waiting for rs1_sim and rs1_mission in the same ROS domain."
        elif phase == "idle":
            message = ("Waiting for " + ", ".join(missing) + ".") if missing else (
                "External ROS stack ready. Select an ROI or use the coordinator's configured target.")
            if not missing and not all(ros.get("alignment", {}).get("ready", {}).values()):
                message += " Current localization differs from Gazebo beyond the initial alignment tolerance; review robot status."
        if self.command and connected:
            kind, _ = self.command
            if kind == "start" and phase == "idle":
                phase, message = "preparing", "Start requested; waiting for coordinator status."
            elif kind == "cancel" and (phase not in TERMINAL or not mission.get("cleanup_complete")):
                phase, message = "cancellation_requested", "Cancellation requested; waiting for executor results."
        if ros.get("alignment", {}).get("invalid"):
            message = "Simulation clock reset. Restart the simulation and mission launches to rebuild alignment."
        if self.command_error and self.command:
            message = self.command_error + " Inspect or restart the mission coordinator; the dashboard will not repeat an uncertain command."
        if phase in TERMINAL and mission.get("cleanup_complete"):
            message += " Restart rs1_mission.launch.py to run another mission."
        return {"mode": "external", "phase": phase, "message": message,
                "can_start": can_start, "configured_roi": mission.get("roi") if connected else None}

    def snapshot(self):
        ros = self.adapter.snapshot()
        return {**copy.deepcopy(self.state), **self.view(ros), "ros": ros, "process_running": False}

    def sample_trails(self):
        if self.lock.locked():
            return
        ros = self.adapter.snapshot()
        instance = ros.get("alignment", {}).get("instance_id")
        now = ros.get("sim_time")
        changed = instance and self.alignment_instance and instance != self.alignment_instance
        rolled_back = now is not None and now > 0 and self.last_sim_time is not None and now < self.last_sim_time
        if changed or rolled_back:
            # Re-subscribe to retained TF so old static registrations cannot survive a new simulator.
            self.adapter.close()
            self.adapter = self.adapter_factory()
            self.state["trails"] = {r: [] for r in ROBOTS}
            self.last_sim_time = None
            self.alignment_instance = instance
            return
        if instance:
            self.alignment_instance = instance
        if now is not None and now > 0:
            self.last_sim_time = now
        mission = ros.get("mission", {})
        mission_instance = mission.get("instance_id")
        if mission_instance and mission_instance != self.mission_instance:
            if self.mission_instance is not None:
                self.command = None
                self.command_error = None
            self.mission_instance = mission_instance
        if self.command:
            kind, _ = self.command
            if (kind == "start" and mission.get("state") not in (None, "ready")) or (
                    kind == "cancel" and mission.get("state") in TERMINAL and mission.get("cleanup_complete")):
                self.command = None
                self.command_error = None
        view = self.view(ros)
        if (view["phase"], view["message"]) != (self.state["phase"], self.state["message"]):
            self.transition(view["phase"], view["message"])
        for robot, pose in ros.get("robots", {}).items():
            if robot in ROBOTS and pose.get("fresh") and "x" in pose:
                trail = self.state["trails"][robot]
                if not trail or pose["stamp"] > trail[-1][2]:
                    trail.append([pose["x"], pose["y"], pose["stamp"]])
                    del trail[:-3000]

    async def start(self, plan: ExternalStart):
        async with self.lock:
            ros = self.adapter.snapshot()
            view = self.view(ros)
            if not view["can_start"]:
                raise Conflict(view["message"])
            identity = (ros.get("mission", {}).get("instance_id"), ros.get("alignment", {}).get("instance_id"))
            self.mission_instance = identity[0]
            if plan.roi is not None:
                self.terrain.check_point("ROI", plan.roi, 0.5)
                roi = {**plan.roi.model_dump(), "z": self.terrain.height(plan.roi.x, plan.roi.y)}
                await self.adapter.configure_roi(roi)
            current = self.adapter.snapshot()
            if identity != (current.get("mission", {}).get("instance_id"), current.get("alignment", {}).get("instance_id")):
                raise Conflict("ROS nodes restarted while configuring the target. Review the current ROI before starting.")
            if not self.view(current)["can_start"]:
                raise Conflict(self.view(current)["message"])
            self.command = ("start", time.monotonic())
            self.command_error = None
            try:
                await self.adapter.call("start")
            except RequestUncertain as exc:
                # Keep the request latched: a service timeout does not prove it was rejected.
                self.command_error = str(exc)
                raise
            except Exception:
                self.command = None
                raise
            return self.snapshot()

    async def cancel(self):
        async with self.lock:
            ros = self.adapter.snapshot()
            self.check_cancel_owner(ros)
            mission = ros.get("mission", {})
            if not ros.get("checks", {}).get("mission_status"):
                raise Conflict("Mission coordinator is unavailable; use its terminal to inspect the ROS stack.")
            if mission.get("state") == "cancellation_requested" or (self.command and self.command[0] == "cancel"):
                return self.snapshot()
            if mission.get("state") not in ("executing", "manual_assistance") and not (
                    mission.get("state") == "failed" and not mission.get("cleanup_complete")):
                raise Conflict("There is no running external mission to cancel")
            self.command = ("cancel", time.monotonic())
            self.mission_instance = mission.get("instance_id")
            self.command_error = None
            try:
                await self.adapter.call("cancel")
            except RequestUncertain as exc:
                self.command_error = str(exc)
                raise
            except Exception:
                self.command = None
                raise
            return self.snapshot()

    def check_cancel_owner(self, ros):
        # External mode operates the coordinator selected by the operator's ROS
        # domain. Deploy mode narrows this to its recorded deployment identity.
        pass

    async def reset(self):
        async with self.lock:
            self.state["trails"] = {r: [] for r in ROBOTS}
            self.state["events"] = []
            # This operation deliberately leaves the ROS connection, pending commands,
            # mission state, and all external processes untouched.
            return self.snapshot()

    async def launch(self, directory):
        raise Conflict("External mode never launches ROS processes")

    async def close(self):
        self.adapter.close()
