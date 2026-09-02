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

- **Missing `camera_rgb_optical` frame**: the robot URDF defines
  `camera_rgb` but not the optical child frame that
  `camera_cone_detector_node` waits for (its `~camera_frame` param default).
  `guidance_gazebo.launch` publishes the standard optical static transform
  as a workaround. Long-term fix: add the optical frame to the URDF.
  **Ask Ethan how this worked on his machine.**
- **control_node startup crash (unresolved)**: in our runs the control node
  dies before writing a log. Prime suspect was 2-waypoint creep paths
  breaking a spline fit in the controller; creep paths now publish 6
  waypoints to preempt that. If it still crashes: run the control node
  manually (`rosrun qcar_control <node>.py`) with the sim up and read the
  traceback.
- **Replanning vs controller timing**: trajectories are republished at
  `plan_rate_hz` with `waypoint_times` restarting at 0. If the car
  stutters or lags, check how the controller treats a replacement
  trajectory mid-execution.
- **Nearest-index cone pairing** is a deliberate simple heuristic; Delaunay
  triangulation is the noted future-work upgrade for sparse/uneven cones.

## Dependencies

`rospy`, `numpy`, `scipy` (`apt install python3-scipy python3-tk` in the
Noetic container), plus workspace packages `qcar_guidance` (for
`TrajectoryMessage`) and `qcar_navigation` (for `ConeDetectionArray`).
