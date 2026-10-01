"""Robot creation must wait for a running world, not cold-start wall time."""

import asyncio
import importlib.util
from pathlib import Path

import pytest
import rclpy
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, SetLaunchConfiguration
from rclpy.context import Context
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


@pytest.mark.parametrize("robot", ["husky", "parrot"])
@pytest.mark.parametrize("simulated_time", [True, False])
def test_create_waits_for_simulated_time(monkeypatch, robot, simulated_time):
    # Keep fake time out of any running simulation and the ROS daemon's domain.
    monkeypatch.setenv("ROS_DOMAIN_ID", "96")
    path = Path(__file__).resolve().parents[1] / "launch" / f"spawn_{robot}.launch.py"
    spec = importlib.util.spec_from_file_location(f"spawn_{robot}_readiness", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    async def exercise():
        created = asyncio.Event()
        ready = asyncio.Event()

        def node_stub(**kwargs):
            # Execute the actual launch/timer graph without Gazebo, xacro or bridges.
            if kwargs["executable"] == "create":
                return OpaqueFunction(function=lambda context: created.set())
            return OpaqueFunction(function=lambda context: None)

        monkeypatch.setattr(module, "Node", node_stub)
        service = LaunchService()
        service.include_launch_description(LaunchDescription([
            SetLaunchConfiguration("use_sim_time", str(simulated_time)),
            SetLaunchConfiguration(f"{robot}_spawn_delay", "0.2"),
            *module.generate_launch_description().entities,
            OpaqueFunction(function=lambda context: ready.set()),
        ]))
        launch_task = asyncio.create_task(service.run_async(shutdown_when_idle=False))
        publisher_context = None
        node = None
        try:
            await asyncio.wait_for(ready.wait(), timeout=5)
            if not simulated_time:
                # Explicit wall-clock mode still works without any /clock publisher.
                await asyncio.wait_for(created.wait(), timeout=3)
                return

            # More than three spawn delays pass during a simulated cold world load.
            await asyncio.sleep(0.7)
            assert not created.is_set(), "Robot creation ran before the world published /clock"

            publisher_context = Context()
            rclpy.init(context=publisher_context)
            node = rclpy.create_node("spawn_readiness_clock", context=publisher_context)
            clock = node.create_publisher(Clock, "/clock", qos_profile_sensor_data)
            for _ in range(100):
                if clock.get_subscription_count():
                    break
                await asyncio.sleep(0.03)
            assert clock.get_subscription_count(), "Launch did not subscribe to simulated time"

            # A live world advancing past the delay must release the create action.
            for tick in range(1, 51):
                message = Clock()
                message.clock.sec, tenths = divmod(tick, 10)
                message.clock.nanosec = tenths * 100_000_000
                clock.publish(message)
                await asyncio.sleep(0.03)
                if created.is_set():
                    break
            assert created.is_set(), "Robot creation did not run after simulated time advanced"
        finally:
            if node is not None:
                node.destroy_node()
            if publisher_context is not None:
                publisher_context.shutdown()
            await service.shutdown()
            assert await asyncio.wait_for(launch_task, timeout=5) == 0

    asyncio.run(exercise())
