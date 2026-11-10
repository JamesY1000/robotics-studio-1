# Software stop authority

Adapted from Nikhil's `feat/ui-functionality` branch (`7270b6c`). The branch's
per-robot/all-robot stop service and dashboard controls are retained, with stop
enforcement moved into the existing velocity selectors to avoid a second relay.

`gz_world.launch.py` starts one `emergency_stop` node for the world. It starts with
both robots stopped, publishes `/safety/state` at 20 Hz and serves `/safety/stop`.
Selectors require a fresh heartbeat and a released latch before forwarding any
command. Any stop pauses the coordinated mission using its existing cancellation
and resume flow, preserving tree measurements and phase progress.

Reset requires the identity/revision displayed to the operator and a fresh
coordinator acknowledgement with `control_mode=hold` and `cleanup_complete=true`.
Each accepted request advances the revision, including repeated Stop requests.
Reset never sends Resume. Restarting this node creates a new identity and latches
both stops again. A missing authority makes running selectors send zero within
approximately 0.55 s of the last heartbeat.

The coordinator must be running before reset is possible, including for standalone
navigation. See [dashboard usage](../../../mission_control/README.md#software-e-stop)
for startup, manual assistance and reset/resume instructions.

This software mechanism is for the simulation. It does not cut power, land Parrot
or guarantee stopping if the selector/bridge/simulator/OS fails. Physical robots
still need hardware safety controls.

Regression coverage includes real ROS services, node restart, stale resets,
missing heartbeat, controller rearming and stop/resume in all three mission
phases. Build with `colcon build --packages-up-to rs1_sim rs1_mission`, then run
the safety, navigation and mission tests in an isolated ROS domain.
