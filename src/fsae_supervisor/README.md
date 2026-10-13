# fsae_supervisor

> **Integrated build for FYP_Part_B (Oct 2026).** The node consumes the
> team's own topics/types (no teammate code changed). Quick start, in a
> catkin workspace containing this package + FYP_Part_B root `src/`:
>
> ```bash
> catkin_make && source devel/setup.bash
> roslaunch fsae_supervisor integration_gazebo.launch track:=Track1        # team sim + supervisor
> rosservice call /mission_manager/set_mission "command: go"
> roslaunch fsae_supervisor integration_gazebo.launch fault_injector:=true # + supervisor-side fault injection
> rosservice call /fault_injector/set_fault "{subsystem: camera, type: '', value: 0, duration_s: 3, active: true}"
> roslaunch fsae_supervisor supervisor.launch platform:=qcar_hw            # hardware (profile is a placeholder)
> roslaunch fsae_supervisor integration_gazebo.launch \
>     extra_config:=$(rospack find fsae_supervisor)/config/enable_filter_gap.yaml   # opt-in r13
> ```
>
> Localisation channels read `/odometry/local` (nav_filter_node); the SLAM
> output `/odometry/filtered` is used only by the opt-in **r13 `filter_gap`**
> channel (13-channel residual vector; design in INTEGRATION_TODO.md §2a).
> `adjust_slam` targets ekf_slam_node.
>
> Platform topic names: `config/platforms/{gazebo,qcar_hw}.yaml`.
> Disabled channels, teammate asks, hardware checks: **INTEGRATION_TODO.md**.
> Accuracy corrections to the port: top of **PORT_NOTES.md**.
> The diagram/table below describe the original port; the
> `synthetic_driver_node` and `full_stack.launch`/`synthetic_test.launch`
> were removed (they used the old custom input messages).

ROS Noetic port of the MATLAB model-based supervisor + mission
manager for the FSAE-Driverless QCar platform.

This package implements the three-stage pipeline described in the
MATLAB reference (`computeResiduals.m` → `imm.m` → `actionPolicy.m`)
plus a mission FSM (original ROS code) and the
fault injector (`faultInjection.m` / `initFaultConfig.m`). IMM math
matches the MATLAB; mode parameters follow the agreed 6-mode design (see PORT_NOTES); the ROS layer handles
I/O only.

```
           ┌──────────────────────┐
sensor     │  fault_injector_node │  intercepts *_raw → *
drivers ──►│  (faultInjection.m)  │◄── ~set_fault service
           └──────────┬───────────┘
                      │
                      ▼
           ┌──────────────────────┐
           │   supervisor_node    │  Stages 2–4, 20 Hz timer
           │  (computeResiduals + │
           │   imm + actionPolicy)│──► /supervisor/action
           └──────────┬───────────┘    /supervisor/posterior
                      │                 /supervisor/residuals
                      ▼                 /supervisor/fault_flags
           ┌──────────────────────┐
           │  mission_manager_node│  FSM, lap counting
           │  (updateMission.m)   │──► /mission/status
           └──────────────────────┘    /mission/target_speed
```

## What's in the package

| Path | Purpose |
|---|---|
| `src/fsae_supervisor/config_loader.py` | Port of `supervisorConfig.m` — builds per-mode sigma matrices from `inflate_idx`. |
| `src/fsae_supervisor/residuals.py` | Port of `computeResiduals.m`, all 12 residual channels + hard-override accumulators. |
| `src/fsae_supervisor/imm.py` | Port of `imm.m` — log-domain Gaussian IMM with Z_CLAMP=5. |
| `src/fsae_supervisor/action_policy.py` | Port of `actionPolicy.m` — hard-override latch, coarse mapping, hysteresis gate, argmax isolation. |
| `src/fsae_supervisor/ring_buffer.py` | Port of the MATLAB `ringPush` / `buf.*_idx` / `buf.*_n` idiom. |
| `scripts/supervisor_node.py` | ROS node running Stages 2–4 on a 20 Hz timer. |
| `scripts/mission_manager_node.py` | Mission FSM; replaces SPACE/ESC keyboard callbacks with a service. |
| `scripts/fault_injector_node.py` | Intercepts `*_raw` sensor topics, applies faults, republishes. |
| `config/supervisor.yaml` | YAML port of `supervisorConfig.m`. |
| `config/fault_injector.yaml` | YAML port of `initFaultConfig.m` including all 7 presets. |
| `config/mission.yaml` | YAML port of the `missionCfg` block in `run.m`. |
| `msg/*.msg` | 9 custom messages (residuals, posterior, action, flags, mission status, MPC stats, EKF state, encoder, cones). |
| `srv/SetFault.srv`, `srv/SetMission.srv` | Runtime control services. |
| `test/test_*.py` | 20 offline tests — the ROS-free sibling of `test_supervisor_isolated.m`. |

## Build

```bash
cd ~/catkin_ws/src
ln -s /path/to/fsae_supervisor .       # or git clone
cd ..
catkin_make                              # or catkin build
source devel/setup.bash
```

`catkin_make` generates `fsae_supervisor.msg` and `fsae_supervisor.srv`
Python modules from the `.msg` / `.srv` definitions; after building,
the nodes in `scripts/` can import them.

## Run

### Supervisor only (against an existing stack or rosbag)

```bash
roslaunch fsae_supervisor supervisor.launch
```

Assumes something else is publishing `/sensors/imu`,
`/sensors/encoder`, `/perception/rgbd/cones`,
`/perception/lidar/cones`, `/sensors/lidar/scan`, `/nav/ekf_state`
and `/control/mpc_stats` — see the `topics:` block of
`config/supervisor.yaml` for the complete list (and remap targets).

### Full stack (supervisor + mission + fault injector)

```bash
roslaunch fsae_supervisor full_stack.launch preset:=cascade mission_mode:=lap
```

The `preset` argument selects one of
`{none, camera_loss, lidar_loss, both_sensors, imu_freeze, steering, cascade, obstacle_on_track}`
— exactly the seven presets in `initFaultConfig.m`.

### Starting a run

```bash
rosservice call /mission_manager/set_mission "command: 'go'"       # start
rosservice call /mission_manager/set_mission "command: 'estop'"    # abort
rosservice call /fault_injector/arm                                # arm scripted faults now
rosservice call /fault_injector/set_fault "{subsystem: 'imu', type: 'freeze', active: true, duration_s: 2.0}"
```

## Test

```bash
# Offline unit tests (ROS not required)
cd fsae_supervisor
PYTHONPATH=src python3 -m pytest test/ -q
```

The suite covers:

- **IMM math** — posterior normalisation, z-clamp sanity, marginalisation of invalid residuals, loglik vs a scalar reference implementation.
- **Residual generator + IMM + action policy end-to-end** — port of the 19 scenarios in `test_supervisor_isolated.m`: nominal baseline, cam/lidar dropout, IMU freeze, drive stall, steering fault, obstacle-imminent, cones-both-lost, single-frame flicker, MPC transient.
- **Action policy in isolation** — hysteresis counters, Degraded→Critical escalation, override latch persistence.

All 20 tests pass on the default config.

## Residual vector (unchanged from MATLAB)

| idx | name | unit | notes |
|---|---|---|---|
| 1 | sigma_pos | m | `sqrt(trace(P[1:2,1:2])/2)` |
| 2 | cte | m | cross-track error magnitude |
| 3 | speed_dev | m/s | `|v_hat - v_ref|` |
| 4 | NIS_mean | - | rolling mean of EKF NIS |
| 5 | mpc_time_norm | - | mean `compute_ms / budget` |
| 6 | imu_enc_parity | rad/s | `|omega_imu - (v_enc/L)·tan(delta)|` |
| 7 | steer_parity | rad | QCar has no servo feedback → binary surrogate |
| 8 | drive_parity | m/s | `|v_ref - v_enc|` — soft signature only |
| 9 | cam_age | s | time since last RGBD cones |
| 10 | lid_age | s | time since last LiDAR cones |
| 11 | imu_freeze | - | `log10(eps / rolling_var(omega))` |
| 12 | enc_freeze | - | same for encoder velocity |

## Mode set (unchanged from MATLAB)

| Fine | Name | Coarse | Speed | Inflates | k |
|---|---|---|---|---|---|
| 1 | Nominal | N | 1.00 | — | 0 |
| 2 | Degraded-Perception | D | 0.50 | r9, r10 | 2 |
| 3 | Degraded-Proprioception | D | 0.50 | r1, r6, r11, r12 | 4 |
| 4 | Degraded-Compute | D | 0.70 | r5 | 1 |
| 5 | Degraded-Actuation | D | 0.50 | r7, r8 | 2 |
| 6 | Critical | C | 0.00 | all | 12 |

Hard overrides (FMEA-mandated immediate-Critical) bypass the IMM:
`cones_both_timeout`, sustained `steering_fault`, drive-stall
signature, `obstacle_detected`.

## Tuning

All tuning lives in `config/supervisor.yaml`. The ones you'll actually
touch:

- **`residual.sigma_nom`** — per-channel nominal std-dev. Raise to tolerate more noise, lower for sharper detection.
- **`mode.inf_deg`** / **`mode.inf_crit`** — sigma-inflation factors. `inf_deg=10` is the Gaussian-over-evidence ratio from Blanke et al.
- **`imm.TPM`** — transition probabilities. Increase self-loops to trust the previous mode more.
- **`action.thresh_{degraded,critical}`** and **`k_enter/exit_*`** — the hysteresis-gated decision thresholds.
- **`override.*`** — hard-override thresholds.

Change them and the tests still pass so long as the mode ordering
and coarse mapping stay consistent.

## Traceability to the MATLAB reference

Every Python file has a one-line docstring that names the MATLAB
file it ports. The same channel indices, the same 6 modes, the same
TPM, the same Z_CLAMP=5 robust clamp, the same hard-override list,
the same 3/2/8 hysteresis counters. See `PORT_NOTES.md` for the
line-by-line audit.

## Platform I/O mapping (QCar)

See the "QCar hardware I/O mapping" table in `fault_dictionary.md`
for the full channel list. The ROS port doesn't know about QCar
channels directly — it consumes standard ROS topics
(`sensor_msgs/Imu`, `sensor_msgs/LaserScan`, the custom
`WheelEncoder` and `ConeArray`) that the driver layer is expected
to publish.
