# qcar_guidance_thomas

Online guidance for the QCar — FYP Part B (Thiha "Thomas" Thet Zaw).

Plans a local path **online** from live fused cone detections
(`qcar_navigation`), instead of loading a pre-known track map — the Part A
limitation this package exists to fix.

```
qcar_navigation (camera+lidar fusion) --> qcar_guidance_thomas --> qcar_control
```

## Contents

| File | Purpose |
|---|---|
| `scripts/online_guidance_node.py` | Main planner: cone memory, midpoint planning, curvature speed profiling, CREEP/TRACK modes |
| `scripts/fake_cone_publisher.py` | Fake curved-track cone data for isolated testing (no Gazebo needed) |
| `scripts/path_visualizer_node.py` | Publishes the planned path as an RViz Marker (`/guidance_path_marker`) |
| `scripts/trajectory_logger_node.py` | Logs car position + trajectory stats to `~/guidance_log_*.csv` for report plots |
| `config/guidance_params.yaml` | All tunables — edit here, no code changes |
| `launch/test_guidance_isolated.launch` | Guidance + fake cones only. Fast iteration |
| `launch/guidance_gazebo.launch` | Full Gazebo sim with this guidance + fixes |
| `extras/setup_workspace.sh` | One-time workspace setup (see below) |

## Install into a fresh UON-QCAR-BASE workspace

1. Copy the whole `qcar_guidance_thomas` folder into the workspace's `src/`:
   ```
   <workspace>/src/qcar_guidance_thomas/
   ```
2. From the **workspace root** (the folder containing `src/`):
   ```bash
   bash src/qcar_guidance_thomas/extras/setup_workspace.sh
   catkin_make
   source devel/setup.bash
   ```
3. Test in isolation first (no Gazebo):
   ```bash
   roslaunch qcar_guidance_thomas test_guidance_isolated.launch
   ```
   Expect: `Mode change: CREEP -> TRACK`, then repeating
   `[TRACK] N waypoints | mem: blue=X yellow=Y | v: 0.20-0.50 m/s`.
4. Full simulation:
   ```bash
   roslaunch qcar_guidance_thomas guidance_gazebo.launch
   ```

The setup script does three things: makes scripts executable, generates
`Track1ExampleThomas.launch` (stock track launch with the guidance node
swapped to this package), and patches a startup crash in
`depth_cone_detector_node.py` (empty `frame_id` guard).

## Topics

Subscribes:
- `/cone_detections_fused_coloured` (`qcar_navigation/ConeDetectionArray`)
- `/odometry/filtered` (`nav_msgs/Odometry`)

Publishes:
- `/qcar/trajectory_topic` (`qcar_guidance/TrajectoryMessage`)
- `/guidance_path_marker` (`visualization_msgs/Marker`, via visualizer)

## Known issues / integration notes

### Resolved
- **`camera_rgb_optical` frame (FIXED by Ethan, Sep 2026)**: the URDF now
  defines `camera_rgb_optical_joint` in `qcar_model.xacro`, so the camera
  detector finds its frame. The static-transform workaround has been
  REMOVED from `guidance_gazebo.launch` -- keeping both would duplicate the
  frame and spam TF_REPEATED_DATA. Re-add it only if running against an
  older base (the line is in a comment in that launch file).
- **depth_cone_detector_node empty `frame_id` crash**: guard applied by
  `extras/setup_workspace.sh`; Ethan has since merged it into his copy, so
  the script now detects it and skips.

### Open -- camera FOV misses the inner boundary (for Ethan)
Measured 15 Sep 2026 on Track1. The camera never sees a single yellow cone,
so colour-based planning cannot work out of the start box. It is NOT a
colour-threshold bug -- ten captured frames contain zero yellow pixels.

Cause is geometry: `camera.xacro` sets `horizontal_fov` = 1.089 rad
(62.4 deg, so +/-31.2 deg) and `max_range_m` = 3.0. From the spawn pose
(-4.30, 3.00, facing -x) the track curves left immediately, so:
  * blue (outer) cones bend INTO view  -- bearings -55, -26.5, -9.8 deg
  * yellow (inner) cones stay wide     -- bearings +55, +41, +37, +37, +39 deg
Zero yellow cones satisfy both the FOV and the range limit.

Suggested fix on the nav side: widen `horizontal_fov` to ~1.6 rad (92 deg),
which also matches real FSAE practice. Until then this package uses the
geometric fallback below.

### Open -- controller requirements (for Luke)
Tested 2 Sep 2026 against the *example* control node. The car moved under
this guidance for the first time, then the controller crashed. Two things
the real controller must do to work with an online planner:

1. **Handle reaching the end of a trajectory.** The example controller
   indexes past the final waypoint and dies with
   `std::out_of_range: vector::_M_range_check`. It should hold the last
   waypoint / stop instead.
2. **Accept replacement trajectories mid-execution.** This node republishes
   a fresh plan at `plan_rate_hz` (default 2 Hz). The example controller
   latched the first trajectory, printed `Trajectory received...STARTING`
   once, and ignored every update -- it drove one stale 0.5 s creep command
   for 8 s straight. Online replanning cannot work without this.

Interface: `qcar_guidance/TrajectoryMessage` on `/qcar/trajectory_topic`.
The speed profile is encoded in `waypoint_times` (slower segments get
longer time allocations); `velocity` is the mean for reference.

### Other notes
- **Geometric fallback (v3)**: when colour is missing, boundaries are split
  left/right by lateral offset in the car frame instead of by colour. Set
  `use_geometric_fallback: false` in the config to disable. The log line
  shows which source is in use, e.g. `[TRACK/geometry]` vs `[TRACK/colour]`.
- **Nearest-index cone pairing** is a deliberate simple heuristic; Delaunay
  triangulation is the noted future-work upgrade for sparse/uneven cones.
- **cone_fusion startup race**: `canTransform: source_frame lidar does not
  exist` appears for ~1 s at startup, then recovers on its own. Cosmetic.

## Dependencies

`rospy`, `numpy`, `scipy` (`apt install python3-scipy python3-tk` in the
Noetic container), plus workspace packages `qcar_guidance` (for
`TrajectoryMessage`) and `qcar_navigation` (for `ConeDetectionArray`).
