# Integration TODO — fsae_supervisor ↔ FYP_Part_B

Status: **monitor-only**. The supervisor runs in the team's Gazebo stack,
publishes `/supervisor/action` (`speed_scale`, `estop`, mode, isolation),
and drives the mission FSM. Nobody acts on `speed_scale`/`estop` yet.
No teammate node has been changed.

## 1. Assumptions made
- The integrated workspace is FYP_Part_B **root `src/`** (not
  `UON-QCAR-BASE-Thomas/` or `-Ethan/`). *(ERR to verify.)*
- **Live filter**: `qcar_guidance_thomas/launch/guidance_gazebo.launch`
  (the only full-stack root launch file) starts **both**:
  - `nav_filter_node` (via `EKF_navigation.launch`, remapped) → `/odometry/local`
    — wheels+IMU only, used by guidance + `control_node`. **The supervisor's
    localisation channels use this one** (`odom_in`): r1 `sigma_pos`,
    r3 `speed_dev`, future r4 `NIS_mean`. Tracking (r2 `cte`) will come from
    MpcStats, which is computed on this same estimate.
  - `ekf_slam_node` (via `qcar_navigation/cone_slam.launch`) → `/odometry/filtered`
    — cone-SLAM. Used by the supervisor **only** as `odom_ref_in` for r13
    `filter_gap`. The reweight-SLAM output (`SupervisorAction.adjust_slam`,
    set in Degraded-Perception) is meant for **`ekf_slam_node`**.
- Gazebo first; real QCar soon → everything platform-specific is in
  `config/platforms/<platform>.yaml`, selected with `platform:=gazebo|qcar_hw`.
- Mission FSM is owned by ERR (`mission_manager_node.py`).
- Mode set: 6 fine modes (Nominal, Perception, Proprioception, Compute,
  Actuation, Critical) → coarse Nominal/Degraded/Critical (explicit
  `mode.coarse_of` in `config/supervisor.yaml`).

## 2. Channels disabled by config (`supervisor.residual.enabled`)
A disabled channel is forced `r_valid=false` (IMM marginalises it) and its
soft flags are suppressed. Re-enable by flipping it to `true` once the data
exists **and** the topic is set in the platform profile.

| # | Channel | Why disabled | What must be published | Re-enable |
|---|---|---|---|---|
| 2 | `cte` | MPC doesn't publish cross-track error | `MpcStats.cte` on `/control/mpc_stats` | set `mpc_stats_in`, `cte: true` |
| 4 | `NIS_mean` | EKF-SLAM doesn't expose NIS | `EkfState.last_nis` (+`updated_this_step`) on e.g. `/nav/ekf_state` from `ekf_slam_node` | set `ekf_state_in`, `NIS_mean: true` |
| 5 | `mpc_time_norm` | MPC solve time not published (was silently "valid=0" before) | `MpcStats.compute_time_ms`, `budget_ms` (50) | set `mpc_stats_in`, `mpc_time_norm: true` |
| 13 | `filter_gap` | opt-in; thresholds untuned | nothing new (both odometry topics exist) — tune on Gazebo bags, then set `filter_gap: true` or launch with `extra_config:=…/config/enable_filter_gap.yaml` | see §2a |
| 7 | `steer_parity` | no measured steering angle (sim or QCar) | Float64 [rad] measured steering on `steer_meas_in` (Gazebo: joint_states of `base_fl/fr` steering joints would work) | set `steer_meas_in`, `steer_parity: true` |

Consequences while disabled:
- **Degraded-Compute is effectively unobservable** (its only signature channel r5 is off).
- **Degraded-Actuation** relies on r8 (`drive_parity`) only; the
  steering hard-override (`steer_err_critical`) can't fire.
- The drive-stall override needs `throttle_cmd`, which nothing provides
  (MpcStats has no throttle field either) → `drive_fault` never fires on its own.

Channels running on **approximations** (enabled, but revisit):
- r3 `speed_dev` and r8 `drive_parity` use `TrajectoryMessage.velocity`
  from guidance as `v_ref` (`platform.v_ref_source: trajectory`). The MPC
  modulates speed locally (`getLocalSpeed`), so true v_ref differs in
  corners → expect looser fits; switch to `mpc_stats` once MpcStats exists.
- r6 `imu_enc_parity` uses the **commanded** steering (mean of
  `/qcar/base_f{l,r}_controller/command`) as δ.
- **r1 `sigma_pos` is currently INVALID in practice**: `nav_filter_node`
  publishes `/odometry/local` with an all-zero pose covariance, so there is
  nothing to read. The channel is left enabled and turns on by itself once
  `nav_filter_node` fills `pose.covariance` (x, y, yaw entries). **Ask for Ethan.**
- **Future NIS (r4)** should also come from `nav_filter_node` (its
  wheel/IMU update innovation), published as `EkfState.last_nis` on `ekf_state_in`.

### 2a. `filter_gap` (r13) — design
- **What:** relative-motion consistency between `/odometry/local`
  (nav_filter_node) and `/odometry/filtered` (ekf_slam_node).
- **How:** both poses are buffered with header stamps (`history_s`=3 s).
  Each tick, both are linearly interpolated at the *same* two instants
  `t_end = min(last stamp of each)` and `t_end − window_s` (1 s) →
  time-aligned despite different rates/latency. The pose increment over the
  window is expressed in each estimator's own body frame, so the constant /
  slowly varying offset between the drifting local frame and the SLAM map
  frame is **not** counted (it's expected). Then
  `r13 = max(|Δp_local − Δp_slam| / pos_thresh_m, |Δψ_local − Δψ_slam| / yaw_thresh_rad)`
  (0.15 m, 0.10 rad) — dimensionless, ≈1 at threshold; `sigma_nom` = 0.5.
- **Invalid when** either stream is older than `max_age_s` (0.5 s) or the
  window isn't covered yet.
- **IMM signature:** inflated (×`inf_deg`) in **both Degraded-Perception and
  Degraded-Proprioception** (and Critical via "all"). A gap means one of the
  two estimators is wrong, but r13 alone can't say which: wheel slip / IMU
  bias corrupts `/odometry/local` (Proprioception), a bad cone association /
  SLAM jump corrupts `/odometry/filtered` (Perception). Which one wins is
  decided by the other channels (r6, r11, r12 vs r9, r10). If r13 fires on
  its own, the argmax tie-break picked Perception in the live test.
  Not put in Compute/Actuation.
- **Why off by default:** SLAM landmark corrections cause legitimate
  jumps in `/odometry/filtered`, which show up as short r13 spikes. Run
  Track1–3/Figure8 bags and look at the r13 distribution before enabling;
  you may need a larger `window_s` or `pos_thresh_m`.
- Startup gate: when enabled, `odom_ref_in` is added to the required inputs.

## 3. Asks for teammates (nothing changed on their side)
1. **Control (Luke, `qcar_control`)**
   - Publish `fsae_supervisor/MpcStats` on `/control/mpc_stats` each MPC
     step: `compute_time_ms` (wall time of the solve), `budget_ms`=50,
     `saturated`, `delta_cmd`, `v_ref` (the speed actually tracked), `cte`.
     (Alternative: a team-owned msg with the same fields — just tell us
     the name and we add an adapter.)
   - **Honour the supervisor**: subscribe to `/supervisor/action`;
     cap reference speed at `speed_scale × v_ref`; on `estop` command zero
     wheel velocity and hold steering. Optionally also respect
     `/mission/status` (only drive in `Running`). **Until this exists, the
     supervisor cannot slow or stop the car.**
2. **Navigation (Ethan, `qcar_navigation`)**
   - `ekf_slam_node`: publish NIS of each landmark update (or mean NIS
     per step) — `EkfState` on `/nav/ekf_state`, or a Float32 we adapt.
   - Optional: consume `/supervisor/action.adjust_slam` (true in
     Degraded-Perception) to down-weight camera detections.
   - Confirm `/cone_detections_lidar` and `/cone_detections_camera` publish
     **every frame, including empty** — or at least that "no message" means
     "no cones". The supervisor treats both as "no cones" (freshness grows).
3. **Guidance (Thomas, `qcar_guidance_thomas`)**
   - Keep `laps_done` in `/guidance_status` JSON (mission lap counting
     depends on it). A typed lap/waypoint-index topic would be cleaner.

## 4. Hardware (QCar 1) — confirm before first run
`config/platforms/qcar_hw.yaml` is a **placeholder**; every `CONFIRM` there:
- **Encoder**: QCar 1 has one drive-motor encoder, not per-wheel velocity.
  Need topic name, type, units (counts vs rad/s vs m/s), gear ratio /
  counts-per-rev → set `encoder_speed_in` (Float32 m/s) or adapt.
- **Steering**: command topic + type on hardware; QCar 1 has **no steering
  feedback** → `steer_parity` stays off on hw unless an estimate is added.
- **IMU** topic/frame and yaw-rate sign (`/qcar/imu`?).
- **LiDAR** topic, `range_min`, and chassis self-hits: the obstacle
  override fires on any return < 0.15 m → set `scan_range_min` /
  angular mask or it will latch Critical immediately.
- Whether `ekf_slam_node` + the cone detectors run unchanged on hw
  (`camera_cone_detector_hw.yaml` suggests yes).
- Wheel radius (placeholder 0.033 m) and sign conventions.
- Battery / motor current topics (FaultFlags `battery_low`/`overcurrent`
  exist but are never set).
- Re-tune `sigma_nom` on real data (rosbag) before trusting Degraded calls.

## 5. Behaviour added during integration
- **Start-up gate**: the pipeline is not stepped until `imu_in, odom_in,
  cones_cam_in, cones_lidar_in, lidar_scan_in` have each delivered one
  message (`~required_inputs`). Before that `/supervisor/action` has
  `mode_name: Initialising`, `speed_scale: 0`, `estop: false`. (Without it the
  cones-both-timeout override latched Critical before Gazebo started.)
- IMU/encoder are marked invalid if silent > 0.5 s (was never invalidated).
- `~reset` service on `supervisor_node`; `set_mission reset` calls it, so
  a latched Critical can be cleared (it re-latches if the fault persists).
- Fault injection is **supervisor-side only** (`/supervisor_fi/*`); team
  nodes keep real data, so closed-loop fault consequences aren't simulated.
  Steering/drive injections are no-ops on this stack.

## 6. Still to do (ours)
- Port the 19 MATLAB validation scenarios into Gazebo runs (fault presets
  + rosbag) and compare against MATLAB logs.
- Re-tune for the 6-mode set on Gazebo bags (Nominal false-alarm rate on
  Track1–3 / Figure8).
- Decide if the MATLAB 4-mode `supervisorConfig.m` should be updated to
  the 6-mode design so the report, MATLAB and ROS agree.
- Dashboard (rqt_plot config or small viz node).
