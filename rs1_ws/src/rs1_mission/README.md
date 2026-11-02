
## Testing the Boustrophedon Sweep

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
ros2 launch rs1_mission roi_sampling_viz.launch.py roi_x_min:=0 roi_y_min:=-90 roi_x_max:=80 roi_y_max:=10
```

``` bash
ros2 run rs1_mission test_roi_sampling.py --roi 0 -90 80 10 --timeout 2400
```

### What flies the sweep
The lanes are flown by **Nav2**, one `NavigateToPose` goal per waypoint rather
than a single `NavigateThroughPoses` for the whole box. A whole-box plan needs the
costmap to be valid across the entire ROI, which it never is at the start; a
one-leg plan only needs the map valid one leg ahead. When a leg fails (empty /
unknown costmap), the executor falls back to the weighted frontier explorer
(`ExploreToPose`, the same exploration used in phase 1) targeted at the *next*
waypoint, so the costmap grows along the lane it is about to fly, then resumes
from where it left off. Terrain following follows the sweep because it watches
the `navigate_to_pose` action status, so the search-clearance descent the
executor requests via `/parrot1/set_clearance` takes effect while the sweep is
executing.

### Sweep start: fly in from the nearest corner (default)

No teleport by default. The executor picks the ROI corner nearest the drone and
starts the boustrophedon from there (lane order and lane direction are chosen to
match), so the run begins with the shortest possible transit instead of crossing
the whole box. It logs the chosen corner and the transit distance:

```
Sweeping from the ROI corner nearest (90.0, -48.0): 23.4 m transit to the first waypoint (100.0, -3.0)
Entered the ROI; starting the sweep lanes
```

The inbound leg is not treated as "leaving the ROI" (`roi_enter_margin`); only the
leg itself is time-bounded (`roi_transit_timeout`, 120 s). It also flies at
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
map. That matters because `parrot1_odom` is the EKF output, which fuses the exact
Gazebo pose, so it equals `world` numerically. A bad `world <-> parrot1_map`
registration therefore cannot make the drone chase waypoints in the wrong place -
but a corrupt EKF pose still can. `test_roi_sampling.py` verifies `parrot1_map`
still agrees with the (ground-truth) odometry frame before sampling (`--teleport`
additionally re-initialises SLAM **after** odometry has settled and checks the
drone sits on the ROI start). It exits 2 with `parrot1_map is N m away from the
odometry frame` on failure.

#### SLAM is on by default, and it drifts

`robot_bringup.launch.py` declares `slam` with **default `True`**, so a plain
`ros2 launch rs1_sim rs1_sim.launch.py` starts slam_toolbox. That is the usual
reason the map "drifts": slam_toolbox is scan-matching a **3 Hz** sweep taken
*inside* the oak branch band on a highly repetitive forest, which is a
fundamentally ill-conditioned problem on this world (a measured
`parrot1_map -> parrot1_odom` has been as bad as `[72, -15, +164]`).

SLAM drift does **not** degrade tree locations or the sweep: perception anchors
on `parrot1_odom` and the executor plans the lanes in `parrot1_odom`. Nav2's
global costmap is in `parrot1_map`, so the executor re-expresses each sweep
waypoint in the map frame before sending it - but that uses the *latest* map
transform, and the drift cancels out because the robot and the goal are both
shifted by the same `map <-> odom` offset, so the relative planned path is
unchanged. SLAM therefore only affects the displayed map and the frontier
explorer (which frontier-searches the SLAM map directly). SLAM stays **on** by
default (`slam:=true`); there is no longer any reason to disable it for a clean
sweep.

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

## Canopy geometry and sweep altitude

The oak trees in `v2_forest.sdf` are single static meshes. Their geometry (mesh
bbox, scaled by 0.0254) is:

| part   | height above the tree base |
| ------ | -------------------------- |
| trunk + limbs (bark) | -0.1 - 5.5 m |
| branches (canopy)    | 1.2 - 6.5 m |

The drone is **kinematic** (`ignition-gazebo-velocity-control-system`), so it can
never physically jam on a branch. Nav2 drives the sweep through this band: the
costmaps are built from the level lidar ring (`/parrot1/scan`) and the full point
cloud (`/parrot1/scan/points`), and each lane is flown one `NavigateToPose` goal
at a time, so only the lane immediately ahead needs to be plannable. The search
and transit clearances are both 2.0 m (matching phase 1).

## Running With UI

``` bash
ros2 launch rs1_sim rs1_sim.launch.py ui:=true
```

``` bash
ros2 launch rs1_dashboard dashboard.launch.py mode:=deploy
```

http://localhost:8000
