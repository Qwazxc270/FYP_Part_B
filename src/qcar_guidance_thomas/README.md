# qcar_guidance_thomas — v5

Guidance for the QCar, FYP Part B (Thiha "Thomas" Thet Zaw).

- **Mapping lap (lap 1):** the car doesn't know the track. It plans online
  from the cones it can currently see and records its own path.
- **Race laps (lap 2+):** once the car is back at its start position, it
  builds one optimised racing line for the whole lap from Ethan's SLAM cone
  map, with a physics-based speed profile, and follows that.

```
qcar_navigation (Ethan) ──► qcar_guidance_thomas ──► qcar_control (Luke)
   cones + SLAM map            trajectory                steering + speed
```

---

## Quick start

From the workspace root (the folder containing `src/`), inside the container:

```bash
bash src/qcar_guidance_thomas/extras/setup_workspace.sh   # once
catkin_make
source devel/setup.bash
```

**1. Isolated test (no Gazebo, ~3 minutes)** — guidance node + a fake
closed-loop track simulator using the real cone layout:

```bash
roslaunch qcar_guidance_thomas test_guidance_isolated.launch
roslaunch qcar_guidance_thomas test_guidance_isolated.launch track:=Figure8
```

Expected: `LAP 1 complete ... (MAPPING)`, then `RACE LINE BUILT ...`, then
`LAP 2/3 complete ... (RACE)`. The fake sim prints `CONE HIT` if the car
touches a cone. RViz shows the planned path (green) and race line (magenta).

**2. Full Gazebo simulation:**

```bash
roslaunch qcar_guidance_thomas guidance_gazebo.launch
roslaunch qcar_guidance_thomas guidance_gazebo.launch track:=Track3 laps:=5
```

`track` = `Track1 | Track2 | Track3 | CircleTrack | Figure8` (spawn pose is
chosen automatically). `laps:=0` (default) never stops.

**3. Offline test (no ROS at all)** — the full stack on every track in a few
minutes; this is what produced the results below:

```bash
python3 src/qcar_guidance_thomas/extras/offline_test.py --plot
```

---

## How it works

### Mapping lap: local planning

Every 0.5 s, from the cones seen recently:

1. **Remember cones** in a fixed frame for 3 s, merging repeat sightings.
   Each detection is placed using the car's pose *at the moment the sensor
   captured it* (detections arrive ~0.4 s late; using the current pose while
   turning smears cones sideways).
2. **Reject bad data:** frames with >80 cones (perception glitch; Gazebo
   produced one with 227), cones seen only once, and cone pairs whose width
   is far from the measured track width.
3. **Find the two edges** by *chaining* cones along each boundary. Colour
   first (blue / yellow); if colour is missing, by position (left / right of
   the car). Each chain starts from the direction the boundary was already
   going, not the car's heading, so the car goes straight through the
   figure-8 crossover instead of turning into the crossing corridor.
4. **Path:** pair each cone with its nearest cone on the other edge, take the
   midpoints, fit a smooth spline, resample to ~8 points/m.
5. **Speed:** slower where the path bends more (0.2–0.5 m/s).

| Mode | When | Behaviour |
|---|---|---|
| `CREEP` | not enough track visible (start) | straight ahead, slowly |
| `TRACK` | both edges visible | midpoint path + speed profile |
| `EDGE` | only one edge visible (inside of a tight corner) | follow it at half the measured track width |
| `RECOVERY` | no cones for 2 s, or commanded but not moving | crawl |
| `LOST` | no cones for 6 s | stop |

### Race laps: optimised line

**Lap detection.** A lap completes when the car has driven ≥10 m, been ≥3 m
from the start, comes back within 1 m, *and* is heading the same way it
started (so the figure-8 crossover near the start doesn't count).

**Building the line** (takes 0.1–0.6 s):

1. Take Ethan's SLAM map (`/cone_map_markers`; colours are decoded from the
   marker RGB) and the path the car drove on lap 1.
2. Decide **once** which side of the lap-1 path each cone is on. (Recomputing
   it from the evolving line let the line cut through a hairpin apex.)
3. Rebuild each edge as a line joining neighbouring cones, and measure the
   distance to it by casting each point's normal ray onto it. (Projecting
   cones onto neighbouring normals badly under-reads the outer edge on
   tight hairpins.)
4. **Optimise:** each point may slide sideways, staying 0.40 m inside both
   edges. Choose the offsets that minimise the squared second differences of
   the evenly spaced points — a quadratic programme with box constraints,
   solved exactly by an active-set method, re-solved 5 times until it
   settles. This rewards a line that is both smooth and short.
5. Check the line stays ≥0.28 m from every cone (widen the margin and retry
   if not; give up safely if impossible).
6. **Speed profile:** grip limit in corners (v² × curvature ≤ 1.0 m/s²,
   capped at 0.8 m/s), then a backward pass for braking (1.0 m/s²) and a
   forward pass for acceleration (0.5 m/s²), so the car slows *before* a
   corner.

**Following it.** Each cycle the next 4 m of the line, starting at the car,
is published — Luke's MPC treats waypoint 0 as "at the car", so sending the
whole lap would break it. `velocity` is the slowest speed in the next 1 m.

**Frames.** Luke's controller uses `/odometry/local` (smooth, but drifts).
The SLAM map is in the SLAM frame (no drift, but jumps 5–18 cm on
corrections). The node continuously estimates the map→local offset,
smoothed over ~1 s, and re-expresses the published window in the local
frame: drift-corrected but jump-free.

**Safety during race laps** — falls back to local planning when:
- the car is >0.6 m off the line (rejoins under 0.3 m),
- the SLAM pose stops arriving,
- the next 2.5 m of line passes within 0.20 m of a cone the car can *see
  right now* (a map/frame-independent check that the line is where the real
  cones are).

If the race line can't be built (patchy map), it does another mapping lap
and retries. **Without SLAM it does not race at all** (`race_allow_own_map`),
because a line fixed in drifting odometry slides off the cones — this caused
14 cone hits in testing before it was disabled.

---

## Test results

`extras/offline_test.py`, 3 laps per run (1 mapping + 2 race), under
**Python 3.8 / numpy 1.17.4 / scipy 1.3.3 (same as the Noetic container)**.
Each track: 3 random seeds (sensor noise, phantom-frame timing, SLAM jumps)
× 2 odometry drift levels (0.1 and 0.6 °/s) = 30 runs.

| Track | Runs passed | Mapping lap | Race laps | Cone hits | Circuit |
|---|---|---|---|---|---|
| Track1 | 6 / 6 | 93–97 s | 36.0–36.8 s | 0 | 93% |
| Track2 | 6 / 6 | 172–174 s | 72.4–73.0 s | 0 | 98% |
| Track3 | 6 / 6 | 244–246 s | 96.7–97.3 s | 0 | 98% |
| CircleTrack | 6 / 6 | 115–116 s | 50.0–51.0 s | 0 | 92% |
| Figure8 | 6 / 6 | 179–184 s | 60.9–62.0 s | 0 | 104% |

Across all 30 runs: every phantom-frame burst rejected, the live-cone race
check never fired (0 false alarms), 0 fallbacks, 0 recoveries.

"Circuit" = race lap length / real track length, to catch driving the wrong
circuit (earlier versions lapped one lobe of the figure-8, or cut across
Track3's infield, while every other check passed).

**What this does and doesn't prove.** The offline car is a kinematic bicycle
with a pure-pursuit tracker, not Gazebo physics and not Luke's MPC. These
results show the guidance logic works end to end on every track layout; they
don't prove driving performance on the full stack. The ROS node itself was
also run end to end through strict stand-ins for rospy and every message
type (unknown fields raise errors), under Python 3.8.

### Race line vs centreline (from the runs above)

| Track | Length (m) | Curvature energy ∫κ²ds | Peak curvature (1/m) | Est. lap time (s) | Min clearance (m) |
|---|---|---|---|---|---|
| Track1 | 30.7 → 28.7 | 2.33 → 1.78 | 0.63 → 0.59 | 38.4 → 35.9 | 0.39 |
| Track2 | 58.6 → 57.8 | 3.95 → 3.30 | 0.79 → 0.73 | 73.3 → 72.3 | 0.38 |
| Track3 | 78.3 → 77.6 | 15.50 → 13.94 | 1.78 → 1.71 | 97.9 → 97.1 | 0.36 |
| CircleTrack | 43.9 → 40.3 | 1.12 → 1.08 | 0.39 → 0.26 | 54.9 → 50.3 | 0.39 |
| Figure8 | 49.5 → 49.1 | 11.17 → 2.92 | 3.21 → 0.64 | 62.1 → 61.4 | 0.36 |

Centreline → race line. Curvature energy is what the optimisation reduces
(smoother line = less steering, more speed possible). On CircleTrack the
energy barely changes (a circle is already smooth) but the line moves inside
and the lap gets 8% shorter. Estimated lap times use the same speed limits
for both lines; with speed capped at 0.8 m/s, corners rarely limit speed, so
most of the gain comes from the shorter path.

---

## Interface

**Subscribes** (names are parameters)

| Topic | Type | Use |
|---|---|---|
| `/cone_detections_fused_coloured` | `ConeDetectionArray` | camera cones with colour |
| `/cone_detections_fused` | `PoseArray` | lidar+depth cones, no colour |
| `/odometry/local` | `Odometry` | smooth pose (same as controller) |
| `/odometry/filtered` | `Odometry` | EKF-SLAM pose (alignment) |
| `/cone_map_markers` | `MarkerArray` | SLAM map with colours (race line) |
| `/cone_map` | `PoseArray` | SLAM map fallback |

**Publishes**

| Topic | Type |
|---|---|
| `/qcar/trajectory_topic` | `qcar_guidance/TrajectoryMessage` |
| `/guidance_status` | `std_msgs/String` (JSON: phase, mode, lap, deviation…) |
| `/guidance_race_line` | `visualization_msgs/Marker` |
| `/guidance_path_marker` | `visualization_msgs/Marker` (visualiser node) |

`~/guidance_log_<time>.csv` (logger node): pose, speed, phase, mode, lap,
race-line deviation, commanded speed — for report plots.

---

## Notes for the team

**Luke (control)**
- Your MPC already resets `prevIndex` on each new trajectory, which is exactly
  right: every trajectory (mapping and race) starts at the car.
- Race laps request up to 0.8 m/s. In `runControl.cpp`,
  `omega = throttle * 10.0`; if the motor command is wheel speed in rad/s,
  full throttle is only ~0.33 m/s (wheel radius 0.033 m). Worth checking.
- The speed profile is in `waypoint_times`; `velocity` is the slowest speed
  in the next metre, so the scalar target already brakes before corners.

**Ethan (navigation)**
- The front camera FOV fix (1.6 rad) means colours now reach guidance; the
  geometric fallback stays as a safety net.
- The depth detector's phantom frames are filtered here (limit 80/frame),
  but the underlying bug is still in `depth_cone_detector_node.py`.
- `qcar_gazebo/launch/Track2Example.launch` loads
  `config-guidance_params.yaml` (dash instead of slash) and will fail.
  `guidance_gazebo.launch` doesn't use the Example files.

**Everyone**
- Vehicle dynamics (SysID) still need checking against the real QCar before
  hardware testing; gains tuned in Gazebo may not transfer.

---

## Files

| File | Purpose |
|---|---|
| `scripts/guidance_core.py` | all guidance logic, no ROS (tested directly) |
| `scripts/online_guidance_node.py` | ROS wrapper around the core |
| `scripts/track_sim.py` | small track/sensor/car simulator, no ROS |
| `scripts/fake_track_sim_node.py` | ROS wrapper for the isolated test |
| `scripts/path_visualizer_node.py` | planned path → RViz |
| `scripts/trajectory_logger_node.py` | run → CSV |
| `config/guidance_params.yaml` | every tunable, commented |
| `config/guidance.rviz` | RViz layout for the isolated test |
| `launch/guidance_gazebo.launch` | full Gazebo stack, any track |
| `launch/test_guidance_isolated.launch` | guidance + fake sim, any track |
| `extras/setup_workspace.sh` | one-time setup + self-test |
| `extras/offline_test.py` | full offline test on every track |

## Dependencies

`rospy`, `numpy`, `scipy` (in the container: `apt install -y python3-scipy`),
and workspace packages `qcar_guidance` and `qcar_navigation`.
