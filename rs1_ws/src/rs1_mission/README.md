
## Testing the Boustrophedon Sweep

Before launching, make sure no previous run is still holding the DDS shared-memory
ports (a surviving `simulation_session`/`robot_state_publisher`/`ign gazebo` wedges
the next run's Nav2 lifecycle and starves the EKF/lidar streams):

``` bash
./kill_stack.sh --check   # must report "clean"; if not, run ./kill_stack.sh first
```

Terminal 1:

``` bash
ros2 launch rs1_sim rs1_sim.launch.py husky:=false rviz:=false
```
headless:=True if you dont want gazebo

Terminal 2:

``` bash
ros2 launch rs1_perception tree_identification.launch.py
```

Terminal 3:

``` bash
ros2 launch rs1_mission roi_sampling_test.launch.py
```

Terminal 4 - RViz view

``` bash
ros2 launch rs1_mission roi_sampling_viz.launch.py roi_x_min:=62 roi_y_min:=-88 roi_x_max:=92 roi_y_max:=-58
```

Terminal 5 test run:

``` bash
ros2 run rs1_mission test_roi_sampling.py --roi 62 -88 92 -58
```

BIGGER SWEEPS

``` bash
ros2 launch rs1_mission roi_sampling_viz.launch.py roi_x_min:=0 roi_y_min:=-70 roi_x_max:=80 roi_y_max:=10
```

``` bash
ros2 run rs1_mission test_roi_sampling.py --roi 0 -70 80 10 --timeout 2400
```

### What flies the sweep
The lanes are flown by **Nav2**, one `NavigateToPose` goal per waypoint rather
than a single `NavigateThroughPoses` for the whole box. A whole-box plan needs the
costmap to be valid across the entire ROI, which it never is at the start; a
one-leg plan only needs the map valid one leg ahead. Every aimpoint is a
`NavigateToPose` goal in `parrot1_odom`, and altitude comes solely from the
terrain-following node, which follows the sweep because it watches the
`navigate_to_pose` action status.

A leg that fails first gets **one** frontier-exploration retry to grow the
costmap (`max_fallback_hops`), and if that does not make it reachable the waypoint
is **abandoned** and the sweep continues with the next one (see *A waypoint it
cannot reach is skipped* below), so one blocked lane checkpoint cannot stall the
run. The explorer is `ExploreToPose`, the same weighted frontier exploration phase
1 uses.

The sweep is **Nav2-only**: there is no internal velocity follower any more.
Every lane and waypoint is a Nav2 `NavigateToPose` goal, and altitude comes solely
from the terrain-following node.

### Lane checkpoints (`max_leg_length_m`)

Each lane is emitted as a series of checkpoints rather than one long leg: the
executor inserts evenly spaced intermediate goals so that no single
`NavigateToPose` leg is longer than `max_leg_length_m` (default **25 m**, a
parameter of the parrot mission executor). On the 80x100 m ROI a lane is ~106 m,
so it becomes 5 legs of ~21 m instead of one 106 m plan, taking the sweep from 14
to 34 waypoints.

This matters because a single leg has to be plannable across its whole length
from the global costmap. Short legs mean each plan only needs the map over a small
strip, a failure costs one checkpoint instead of a whole lane, and the per-waypoint
stall watchdog has somewhere to resume from mid-lane. Raise it to reduce the number
of goals (and the ~1 s of Nav2 overhead each one costs); lower it for a denser,
more forgiving sweep.

### A blocked waypoint is accepted once it is close enough

A checkpoint the drone physically cannot occupy - a tree sits on the aimpoint, or
the goal lands in inflated space just past a canopy edge - is the common cause of
a stall at a lane end or U-turn. Instead of insisting on the exact goal, the
executor **accepts the waypoint once the drone is within
`waypoint_close_enough`** (default **3.0 m**, including both mission launch files) of it:

* **Already there.** Before sending a leg, `navigateToNextWaypoint()` skips past
  any waypoint the drone already satisfies, so no doomed goal is ever sent.
* **Parked near it.** The stall watchdog takes the waypoint after a short
  `waypoint_close_enough_dwell` (default **3 s**) of sitting still inside that
  radius, instead of waiting out the full 60 s `waypoint_stall_timeout`.
* **Leg aborted.** If Nav2 aborts the leg while the drone is already within
  range, the waypoint is taken and the sweep continues, saving the exploration
  round-trip.

Automatic lane spacing is at most 16 m with the default detection range and
overlap. A larger acceptance radius can shorten lane ends, so check observed
coverage when increasing it. Accepting a waypoint counts as **progress, not a
skip**, so it can never trip the "wedged in one spot" abort. Logged as:

```
Waypoint 6/11 (52.1, -20.8) accepted as reached: 1.42 m away (close-enough 3.00 m; Nav2 aborted the leg)
```

### A waypoint it cannot reach is skipped, not fatal

If a checkpoint cannot be reached *and* the drone is further than
`waypoint_close_enough` from it, the executor **skips that waypoint and carries on
with the next one** instead of aborting the run, because on a long lane losing one
checkpoint is far cheaper than losing the rest of the sweep. Two paths lead there:

* **No progress.** Once a per-waypoint goal has been in flight with the drone
  moving less than `waypoint_progress_epsilon` (1 m) for `waypoint_stall_timeout`
  seconds (default **60 s**), the goal is cancelled and the waypoint skipped. The
  progress signal is the drone's own pose, not Nav2 feedback (bt_navigator keeps
  ticking while the planner is blocked).
* **A failed leg.** A failed leg first falls back to the frontier explorer to grow
  the costmap (`max_fallback_hops`, default **1**). If that still cannot reach the
  waypoint it is skipped. Raise the value for more exploration attempts, or set it
  to `0` to skip the moment a leg fails.

Sampling only aborts if **three waypoints in a row cannot be reached from the same
spot** (`max_consecutive_skips`), which means the drone is genuinely wedged rather
than one lane leg being blocked. Skipping several blocked checkpoints while the
drone still moves down the lane is normal and does not abort the run. The tunable
knobs are launch arguments of `roi_sampling_test.launch.py` (and
`waypoint_close_enough` is also a launch argument of `rs1_mission.launch.py`, so
dashboard runs can pass it):

``` bash
ros2 launch rs1_mission roi_sampling_test.launch.py \
    waypoint_stall_timeout:=30.0      # give up on a stalled leg sooner
ros2 launch rs1_mission roi_sampling_test.launch.py \
    max_fallback_hops:=0              # skip at once, no exploration retry
ros2 launch rs1_mission roi_sampling_test.launch.py \
    waypoint_close_enough:=3.0        # default acceptance radius for blocked aimpoints
```

Skipped waypoints are logged as `Skipping waypoint N/M (x, y): <reason>`, and the
partial tree map still covers every lane that was flown.

### Inference pauses while the drone is stationary

During sampling the executor asks the perception stack to detect at
`detection_rate_hz` (default 1 Hz). It stops asking while the drone is parked: once
`stationary_detection_limit` requests (default **3**) have been issued from the same
spot, the timer issues no more until the drone has moved
`detection_motion_threshold_m` (default **0.5 m**). A moving drone resets the
counter every tick, so normal flight still detects at the full rate.

This matters because a hovering drone re-reports the same trees from the same
viewpoint, and YOLO is the heaviest CPU consumer in the stack — it competes with
Gazebo, SLAM and Nav2 for the same cores. The pause covers waiting at a lane end,
reversing out of a wedge, and the gap between two waypoint goals. Set
`stationary_detection_limit: 0` to pause immediately on stopping, or a large value
to disable the limiter.

### Sweep start: fly in from the nearest corner (default)

No teleport by default. The executor picks the ROI corner nearest the drone and
starts the boustrophedon from there (lane order and lane direction are chosen to
match), so the run begins with the shortest possible transit instead of crossing
the whole box. It logs the chosen corner and the transit distance:

```
Sweeping from the ROI corner nearest (90.0, -48.0): 23.4 m transit to the first waypoint (100.0, -3.0)
Entered the ROI; starting the sweep lanes
```

The inbound leg is not treated as "leaving the ROI" (`roi_enter_margin`), and it
is bounded by Nav2's own goal/progress timeouts. It also flies at
`transit_clearance` and only descends to `search_clearance` once it is inside the
box, so the whole inbound leg is not flown at canopy height:

```
Transiting to the ROI at the transit clearance; the drone will descend to the search clearance on arrival
Sweeping from the ROI corner nearest (90.0, -48.0): 23.4 m transit to the first waypoint (100.0, -3.0)
Entered the ROI; starting the sweep lanes
Descending to the search clearance for the sweep
```

To skip the transit and be placed at the nearest corner instantly, use `--teleport`
(this needs a SLAM re-initialisation after the jump):

``` bash
ros2 run rs1_mission test_roi_sampling.py --roi 0 -90 80 10 --teleport
```

`--no-teleport` is still accepted as a no-op alias.

### If the sweep flies away or leaves the ROI

The sweep plans and controls in **`parrot1_odom`** (`sweep_frame_`), NOT in the SLAM
map, so SLAM's map<->odom wander cannot make the drone chase waypoints in the wrong
place.

`parrot1_odom` is the EKF output and is **not** the same frame as Gazebo's `world`:
with the current EKF config (`odom0_config` fuses `vx, vy, vyaw` only) it is a dead
reckoned frame whose origin is wherever the filter started, so anything expressed in
one frame must be converted, never relabelled. Three places matter:

| Data | Frame it is really in | Converted by |
| --- | --- | --- |
| ROI box (`test_roi_sampling.py`, dashboard) | `world` | the executor, in `prepareSweepBox()` |
| Tree positions (`/parrot1/tree_map`) | `world` (perception uses registered localisation TF) | published with `frame_id: world`; filtered against a `world` copy of the ROI |
| Sweep waypoints | `parrot1_odom` | by construction |

`roi_sampling_viz.py` publishes its markers in `world` for the same reason.

Two guards catch a genuinely broken localisation chain:

* `test_roi_sampling.py` checks the `parrot1_odom -> parrot1_base_link` chain exists
  before sampling, and `--teleport` waits for the drone to reach the target on the
  ground-truth `world` chain, re-initialises SLAM afterwards, and verifies the
  drone's map pose sits on the ROI start.
* Nav2's own goal/progress checker bounds each lane leg, so a leg that cannot be
  completed is abandoned rather than flown away from the ROI.

#### SLAM is on by default, and it drifts

`robot_bringup.launch.py` declares `slam` with **default `True`**, so a plain
`ros2 launch rs1_sim rs1_sim.launch.py` starts slam_toolbox. That is the usual
reason the map "drifts": slam_toolbox is scan-matching a **3 Hz** sweep taken
*inside* the oak branch band on a highly repetitive forest, which is a
fundamentally ill-conditioned problem on this world (a measured
`parrot1_map -> parrot1_odom` has been as bad as `[72, -15, +164]`).

The executor plans lanes in `parrot1_odom` and converts each waypoint to
`parrot1_map` for Nav2. Perception projects detections into `world` through
the registered TF chain, so SLAM drift can shift a tree estimate, split repeated
sightings or move a detection across the ROI boundary.

If you want the map itself to be less noisy, tune
`rs1_localisation/config/parrot/slam_params_parrot1.yml` (`minimum_travel_distance`
is `0.0`, i.e. a map node on almost every scan; `max_laser_range: 40.0` drags in
canopy returns) or build the map by accumulating the lidar in the `parrot1_odom`
frame, which has no drift by construction because that frame *is* the truth.

The README's older claim that "the sweep plans in `parrot1_map`" was stale; the code
sets `sweep_frame_ = "parrot1_odom"`.

Check the chain before sweeping:

``` bash
ros2 run tf2_ros tf2_echo parrot1_odom parrot1_base_link   # must match /parrot1/odometry
ros2 run tf2_ros tf2_echo parrot1_map  parrot1_odom        # identity
ros2 run tf2_ros tf2_echo world        parrot1_map         # ~identity
ros2 topic echo /parrot1/odometry --once --field pose.pose.position
```

Teleop Terminal (cmd vel is husky1 and parrot1):

``` bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/husky1/cmd_vel
```

## When the drone gets stuck in a tree

The oak trees in `v2_forest.sdf` are single static meshes. Their geometry (mesh
bbox, scaled by 0.0254) is:

| part   | height above the tree base |
| ------ | -------------------------- |
| trunk + limbs (bark) | -0.1 - 5.5 m |
| branches (canopy)    | 1.2 - 6.5 m |

The drone is **kinematic** (`ignition-gazebo-velocity-control-system`), so it can
never physically jam on a branch. With the Nav2 sweep, "stuck in a tree" means the
planner/controller cannot find a collision-free path because the level scan ring
(`/parrot1/scan`, the middle ring of the 9-ray lidar at the drone's altitude) sees
branches on every bearing and the costmap closes around the drone.

The only clear altitudes are **below ~1.2 m** (trunk level, where only widely
spaced trunks remain) and **above ~6.5 m** (above the canopy). The mission
coordinator's `search_clearance` / `transit_clearance` default to **2.0 m**
(inside the branch band) but are exposed as launch arguments
(`search_clearance`, `transit_clearance`) and as the
`MISSION_SEARCH_CLEARANCE` / `MISSION_TRANSIT_CLEARANCE` environment variables
for the dashboard launches, so a run that wedges in the canopy can be moved
above it (e.g. 7.0 m) without rebuilding. The sweep relies on Nav2 plus two
behaviours:

* **Explore once, then skip.** When a `NavigateToPose` leg aborts (the costmap
  cannot plan through the canopy) the executor hands the current waypoint to the
  frontier explorer for one round-trip so the map can grow along the lane, then
  retries. A second failure skips the waypoint (`max_fallback_hops:=0` skips at
  once; raise it for more attempts).
* **DWB recovery.** The controller's own `BackUp` / replan cycle in the custom
  behaviour tree (`navigate_to_pose_custom.xml`) clears the costmap and retreats
  when the path is blocked.

There is no duck/pop-up ladder, no gap-seeking avoidance and no `cmd_vel` follower
in the executor any more - all of that was removed when the sweep moved to Nav2,
and `sweep_controller` is no longer a parameter.

### Sweep speed

Speed comes from Nav2's controller (DWB `max_vel_x` in
`rs1_navigation/config/parrot/nav2_params_parrot1.yml`), not from the executor.
The drone is kinematic, so it reaches the commanded speed instantly and there is
no sim-side acceleration limit.

## Running With UI

``` bash
ros2 launch rs1_sim rs1_sim.launch.py ui:=true
```

``` bash
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy
```

http://localhost:8000

## Full mission and the ground path handoff

The coordinator first sends Husky a `TraverseToROI` goal so it can accept
Parrot's ground path. Parrot's traversal succeeds after it reaches the ROI and
Husky acknowledges `SetGroundPath`. At that point the coordinator starts
`/parrot1/sample_roi`. Husky keeps its original traversal goal while Parrot
sweeps. The coordinator starts `/husky1/measure_trees` after **both** Husky's
arrival and a successful sweep with trees inside the original ROI. Either can
finish first. An empty map asks for Parrot assistance and Resume repeats the sweep.

The dashboard's deploy and managed launches include
`rs1_perception/tree_identification.launch.py`. Start it separately in external
mode. The overlapping actions run perception and both Nav2 stacks at the same
time, so check controller and costmap update rates on the deployment machine.

### Tree measurements

Husky visits each pending map index in nearest-neighbour order from its current
pose. It asks `/husky1/locate_tree` for a trunk centre from fresh 3D LiDAR scans,
using a staging pose when the mapped tree is far away. The search is bounded by
1.8 m and half the distance to the nearest other mapped tree. Multiple plausible
trunks or inconsistent centres leave the identity unresolved. A visible lower
stem can guide the approach even when the robot cannot yet see breast height.
After identifying the trunk, Husky first checks whether its current stopped pose
is already suitable for DBH. During autonomous approaches, if range, fresh maps, parked footprint, view-quality restrictions and
settling checks pass, it measures there without sending another navigation goal.
Otherwise it plans an approach from that position without spending a navigation
failure. Resume keeps completed readings and rebuilds the remaining
nearest-neighbour order; it does not force the previous approach pose.
Manual Resume has a dedicated stationary check described below.

Both localisation and DBH require the same coherent track to reach below
0.75 m above ground. Two repeatable foliage rings around a metre up must not
become a new trunk centre. When the lower stem is hidden, Husky needs another
view rather than accepting that foliage as the target.

When a new approach is needed, Nav2 approaches the refined centre, preferring
`tree_standoff_m` (1.5 m by default). The executor also considers wider views when
closer ones are blocked.
Nominal radii are the preferred distance, plus 0.5 m and plus 1.0 m. They are
clipped inside the accepted measurement range by `tree_goal_margin_m` (0.35 m),
giving 1.5 m, 2.0 m and 2.5 m with the defaults. This leaves room for Nav2's
0.30 m position tolerance; a goal at the hard measurement limit could finish outside
the measurement range. Keep this margin above the configured Nav2 tolerance.

Before moving, it checks 32 bearings at each distance against Nav2's published
padded footprint and the latest global costmap. Planning attempts are spread
across eight sides and the available radii. Each round checks up to eight routes
with `ComputePathToPose`, then chooses a reachable one using actual path length,
cost and distance from the preferred stand-off. If a round has no usable route,
it continues up to 24 planning attempts per search. After a failed DBH view,
it can also try a closer radius (1.35 m with the defaults) and up to 32 plans. The route also
passes a footprint check along its segments and the initial turn, so a free goal
cell alone isn't enough. Candidate headings are independent of the trunk direction;
the chosen arrival heading follows the checked route and can face backwards.
Unknown cells are allowed during approach planning, matching Nav2's
`allow_unknown: true`. They receive a small ranking penalty so a known clear
view is preferred. Known obstacles and positions outside the costmap remain
blocked. Nav2 continues checking the live local costmap while driving, since a
prechecked route can still change. A recovery only treats Husky as clear when
its current footprint is observed free, rather than assuming unknown is safe.
If all measurement positions or their paths are blocked, Husky can make one move
around the trunk at the wider acquisition distance (2.8 m by default). It waits
for an updated costmap before checking measurement positions again. This extra
view is for mapping; DBH requires an actual robot-to-trunk distance within
`tree_measurement_min_m` and `tree_measurement_max_m` (1.0–4.0 m). The launch passes
these same limits to the executor and measurement service. Wider views still
need enough visible lower stem and breast-height returns to pass the detector.

Tree approaches use `config/tree_sampling_navigation.xml`, supplied through the
`tree_navigation_bt` parameter by the mission launch. This tree allows one costmap
refresh and retry. The survey owns the remaining recovery: after 12 wall-clock
seconds without 20 cm of movement, it cancels the approach and waits for Nav2's
terminal result before checking its current footprint in both the global and
fresh local costmaps. If Husky is clear, it can use its current measurement view
or try another approach without backing up. A footprint that is clear at the
parked heading can still collide when turning onto a route. If all candidate
routes fail and their footprint checks show a blocked departure within 35 cm of
the start, recovery treats that as needing more turning room. If its footprint
or departure is blocked, it
requests a collision-checked 50 cm retreat at 10 cm/s.
Only one retreat is allowed at the same spot, with at most two per tree. A second
retreat requires at least 1 m displacement from the previous recovery position.
A blocked retreat requests assistance rather
than repeatedly driving against the same obstacle. A partial retreat can also
trigger fresh path checks if Husky moved at least 20 cm and its footprint is now
clear. A tree with no reachable position is deferred without retreat when Husky
itself is in clear space. ROI traversal keeps
its existing behaviour tree.
The relevant executor limits are `tree_progress_timeout_s` (12),
`tree_motion_timeout_s` (60) and `tree_planning_timeout_s` (3).

Sampling selects the `TreeApproach` DWB controller and `tree_goal_checker` in
its own behaviour tree. The position tolerance stays 0.30 m, but every yaw is
accepted and the sampling controller omits `RotateToGoal`. Normal navigation
explicitly selects its existing `FollowPath` controller and `general_goal_checker`.

After an autonomous approach, Husky checks its actual range and parked
footprint in fresh global and local costmaps. It waits for at least 0.5 s of fresh
poses with less than 2 cm translation and 0.03 rad heading drift. There is no Spin
action or face-tree requirement: the target can be beside or behind the 360 degree
scanner. Failed readings exclude candidate positions within 0.60 m of the failed
view, so another attempt changes the view rather than just rotating in place. Each scan uses its own TF timestamp and estimated ground
plane. The service follows trunk widths across LiDAR rings near 1.3 m above
ground. It requires five fresh scans with consistent centre and diameter and
returns their median. For small, round cross-sections it refines the width by
fitting a circle to the individual ring. Its centre tolerance defaults to 0.35 m and tightens for
close neighbours. Its default timeout is 7 wall-clock seconds.

For a width of 0.45 m or more Husky seeks a second view from another side,
at least roughly 45 degrees away from the first viewing axis. If the widths
agree within 10% of the first width it records their mean. Failed approaches and readings trigger
alternative sides. Navigation, trunk localisation and DBH each have a separate
three-failure budget. A navigation failure cannot spend a DBH attempt. Reaching a
localisation or usable measurement view resets the navigation failure count.
After exhausting candidate sides or one of these budgets, Husky defers that
tree and visits the remaining ones. If Nav2 stalls at the
same blocked spot on approaches to two different trees, Husky requests help there.
Unresolved indices keep
the mission in assistance. Only a confirmed reading for every mapped index
completes the mission.

To capture perception failures, pass `dbh_diagnostics_dir:=/path/to/captures`
to the mission launch. Each localisation or DBH request saves its target,
tolerance, detector settings, result, and up to eight fresh scans. Each scan
contains the original serialized PointCloud2, XYZ points, scan-time transform,
ground fit and ring/track rejection details. Successful requests are captured
too, so they can be compared with failures. Capture is off by default and is
bounded to 64 requests per service process (`max_diagnostic_requests`).

### Manual assistance and Resume

Manual assistance stops both autonomous actions and gives the selected robot
manual control. Resume waits for their terminal results and fresh localisation.
Husky’s Nav2 launch enables process respawn so a stopped controller can come
back while the operator is assisting. During tree sampling, Resume checks the
manually chosen position before waiting for Nav2 or costmaps.

| When assistance starts | What Resume does |
| --- | --- |
| Before the ground path handoff | Restarts traversal and requests a new handoff |
| After handoff, while Husky is driving | Continues the saved route from the first unfinished guidance pose |
| During an unfinished Parrot sweep | Restarts the sweep over the same box |
| After the sweep or Husky's arrival | Keeps the completed map or arrival and resumes only unfinished work |
| During tree measurement | Restores completed and unfinished readings, then checks the nearest unmeasured tree from the parked pose |

For assisted sampling, drive within 1–4 m of an unmeasured tree, release the
controls and select Resume autonomous mission. The executor receives
`resume_from_current_pose=true`, identifies the trunk from fresh scans and checks
the 1.0–4.0 m range and 0.5 s settling requirement. Heading does not need to face
the tree. This stationary measurement does not require a clear footprint in the
costmaps or a running Nav2 planner. All normal collision checks still apply to
subsequent autonomous movement. An unusable view returns to manual assistance
with a reason instead of driving away from the operator's chosen position.

The initial map check allows the maximum measurement distance plus the trunk
search uncertainty: up to 5.8 m with the defaults. Nearby mapped neighbours
reduce that uncertainty so the search cannot cross into another tree's region.
Resume tries other nearby unmeasured candidates if the first cannot be located
or measured from this position. Completed measurements and pending second views
remain attached to their original tree indices.

The dashboard highlights the selected tree and displays its current distance,
minimum and maximum range rings. A mapped estimate is labelled separately from
a trunk located in fresh scans. The executor publishes this information on
`/husky1/tree_sampling_target` and logs map distance, correction and failures.
Range failures state the actual distance and how far to move closer or farther
away. A lower-trunk-only scan reports that breast height is obscured instead of
suggesting that another range change will fix it. Wider range acceptance retains
the same fresh-scan, target identity, lower-stem and breast-height checks; it does
not guarantee a usable view at every distance.

After a valid reading, Husky continues autonomously. You can request assistance
again or wait for a navigation failure, reposition and repeat. Completed trees
are excluded from the next nearest-neighbour search. If the nearest remaining
tree cannot be identified or measured, the reading is not credited to another
visible trunk.

Large trees keep their first reading in `TreeSurvey.pending_views`, including the
tree index, diameter, trunk centre and viewpoint in the tree map frame. The
executor returns these on cancellation or failure and the coordinator sends them
back on Resume. Measuring another tree or deferring the large tree preserves
this state. Return from another side of the large tree, roughly 45–135 degrees
from its first bearing, to finish it. A repeated bearing or disagreement over
10% leaves the first reading pending. Only the confirmed two-view average enters
`measurements`; pending views do not count toward mission completion.

This progress survives measurement action restarts while the coordinator remains
running. It is not a checkpoint for restarting the entire mission stack.

### Phase 3: return to the start

Once **every Husky DBH is confirmed**, the coordinator sends independent
`/husky1/return_home` and `/parrot1/return_home` actions. Both executors use Nav2's
normal `NavigateToPose` behaviour and collision checks. The mission stays
`returning` until both robots succeed; it does not finish when only Parrot's
sweep or Husky's measurements finish. Measurements and the AGB summary remain
available during the return and after completion.

Dashboard deploy and managed launches pass each robot's actual saved spawn
`[x, y, z, yaw]` as `husky_home` and `parrot_home`, with `home_from_start=false`.
Standalone launches default to `home_from_start=true`, capturing each robot's
fresh world-frame pose once when Start Mission is first accepted. Resuming never
overwrites these targets. To use explicit standalone spawn poses, pass:

```bash
ros2 launch rs1_mission rs1_mission.launch.py home_from_start:=false \
  husky_home:='[90.0, -48.0, 0.0, 0.0]' parrot_home:='[92.0, -48.0, 4.0, 0.0]'
```

Each attempt transforms the saved world pose into that robot's current map frame.
Nav2 controls XY and heading; terrain following controls Parrot's height. Parrot
returns above its starting position at transit clearance, **without landing**.
After Nav2 succeeds, fresh localisation must confirm XY within
`home_tolerance_m` (0.5 m by default). A robot already within that distance can
complete without another navigation goal.

Failure, missing localisation, rejection or 30 seconds without 20 cm of motion
requests manual assistance. A return attempt also has a 600-second overall
limit; the executor parameters are `home_progress_timeout_s` and `home_timeout_s`.
The coordinator holds both robots and cancels outstanding actions. Resume waits
for terminal action results, then retries the unfinished home journeys from the
robots' current positions. An already-arrived robot stays parked, unless the
operator has driven it away from home. Previous sampling is never repeated.

Cancel mission & return home stops the current traversal, sweep or measurement,
waits for its final action result, then sends both robots to their saved starts.
Confirmed DBHs and pending second views are preserved. The mission stays active
through the return, including repeated manual assistance and Resume, and ends as
`cancelled` only after both robots arrive. Normal phase 3 still ends as `complete`.
Repeated Cancel requests during a return do not interrupt it or resume a paused
return. Use the software E-stop to stop movement immediately, or Manual assistance
to reposition a robot. Stop robots stays blocked until the mission is terminal
and action cleanup is complete. Cancelling managed launch preparation still stops
that launch because no mission or home journey has started yet.

Nikhil's `feat/return-to-home` branch supplied the saved-home and phase-resume
approach and the launch-test foundation. This version updates the phase-2
transition for confirmed Husky DBHs, uses separate home actions and poses, and
keeps the distinct `returning` status supported by the current velocity selector.

Route checkpoints are held in the running Husky executor and are tied to the
mission's route ID and ROI. The coordinator publishes the filtered map on
`/mission/tree_map` and the survey on `/mission/tree_survey` when the map is
ready and when the measurement action returns, including partial results.

```bash
ros2 interface show rs1_interfaces/srv/LocateTree
ros2 interface show rs1_interfaces/srv/MeasureDBH
ros2 topic echo /mission/tree_survey --qos-durability transient_local --once
```

### Sampled above-ground biomass (AGB)

The mission launch also starts `carbon_calculator_node.py`. It subscribes to
`/mission/tree_survey` and recomputes AGB from confirmed `measurements` whenever
the coordinator publishes a snapshot: when the map is ready and when the
measurement action returns, including pauses and partial results. It does not
read the ground-truth CSV or fill missing DBHs with defaults. Pending first views
of large trees do not count until the second view confirms their diameter.
Replaying the same snapshot or resuming assistance cannot add a tree twice.

The existing model is `AGB = 0.0673 * (rho * D_cm^2 * H_m)^0.976`, with DBH
converted from metres to centimetres. `config/tree_carbon.yaml` supplies nominal
height and wood density: oak 12 m / 0.71 g/cm³ and pine 10 m / 0.545 g/cm³.
Height and density are not measured by this mission. The
[Chave et al. (2014) model](https://onlinelibrary.wiley.com/doi/10.1111/gcb.12629)
was developed for tropical trees; these configured oak/pine estimates are a demo
model, not a calibrated biomass result for this forest. Unsupported species are
excluded and reported in the summary rather than assigned a default model.

`/carbon/summary` is a latched `BiomassSummary`, containing the mission ID,
source tree-map header, estimated total, coverage counts and validity message.
Mission Control uses both identities to keep previous mission totals out of the
current ROI. Its label says **Sampled AGB estimate** until every mapped tree has
an estimate, then **Total AGB estimate** with translucent green ROI shading.
The label includes the number of trees contributing. An empty survey displays
**Awaiting DBH samples**, not zero biomass. Hover over the label for the model
assumptions and any unsupported tree indices.

For compatibility, the node also publishes `/carbon/total_agb_kg` (`Float64`)
and per-tree `/tree_dbh` records. The numeric topic is zero for an empty survey
and NaN for invalid input; consumers needing coverage and identity should use
`/carbon/summary`. `/calculate_carbon` (`Trigger`) recomputes the latest survey;
no service call is needed during normal operation.

```bash
ros2 topic echo /carbon/summary --qos-durability transient_local --once
ros2 service call /calculate_carbon std_srvs/srv/Trigger '{}'
```

For an existing survey without the normal mission launch, run
`ros2 launch rs1_mission tree_carbon.launch.py use_sim_time:=true` once. Do not
start a second calculator alongside the one already included in the mission.
