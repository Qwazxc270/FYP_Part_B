# Port notes — MATLAB → ROS Noetic

> **Integration audit (Oct 2026) — corrections to the original notes.**
> The original text below overstated equivalence. Corrected facts:
>
> 1. **Mode set.** The current MATLAB `supervisorConfig.m` implements a
>    **4-mode** set (Nominal, Degraded-Sensing, Degraded-Vehicle,
>    Critical) and the MATLAB `README.md` describes a 7-mode set. This
>    port implements the **6-mode** set from the report (Nominal,
>    Perception, Proprioception, Compute, Actuation, Critical), which is
>    the **agreed design**. TPM, `mu0`, signatures and speed scales
>    therefore do *not* match today's MATLAB; "byte-for-byte" applies
>    only to the IMM *math* (log-domain likelihood, Z_CLAMP, floors),
>    not to the parameters. The MATLAB Sensing mode also uses non-zero
>    signature means on r9–r12; the port uses zero means + inflation.
> 2. **Coarse mapping** is now explicit (`mode.coarse_of` in
>    supervisor.yaml), validated at load, and `ActionPolicy` refuses a
>    config that isn't the 6-mode N,D,D,D,D,C layout.
> 3. **Presets:** there are **8** presets incl. `none`, not 7.
> 4. **Mission FSM:** no `initMission.m` / `updateMission.m` /
>    `getMissionTargetSpeed.m` exist in the MATLAB sources supplied;
>    `mission_manager_node.py` is original ROS code, not a port. It now
>    counts laps from the team guidance node's `/guidance_status`.
> 5. **Auto-arm:** the fault injector auto-arms whenever `~auto_arm`
>    is true (default), not only for presets with `start_time <= 0.05`.
> 6. **Steering/drive injection** sets state only; nothing feeds it to
>    r(7)/r(8) on the team stack (no steering feedback, no MpcStats).
> 7. **Test coverage:** `test/` contains unit tests (47 after
>    integration), *not* re-runs of the 19 listed MATLAB scenarios.
> 8. **Bugs fixed:** `latch=True` passed to `rospy.Subscriber`
>    (TypeError at startup) in mission_manager; mission_manager param
>    lookup crashed under `full_stack.launch`; `package.xml` lacked
>    `rospy`/`std_srvs` build_depends (catkin configure error);
>    `mpc_time_norm` was always "valid" (pushed 0.0 every tick even with
>    no MPC data) — now masked by config.
> 10. **New channel r13 `filter_gap`** (not in MATLAB): `/odometry/local`
>    vs `/odometry/filtered` relative-motion discrepancy, opt-in. The
>    residual vector is now 13 long; `ResidualVector` arrays are
>    variable-length so the message is unchanged. Localisation channels
>    (r1, r3, future r4) now read `/odometry/local` (nav_filter_node)
>    instead of the SLAM output. Design in INTEGRATION_TODO.md §2a.
> 9. **Interfaces:** the custom input msgs (`ConeArray`, `WheelEncoder`,
>    `EkfState`, `MpcStats`) are not published by the team stack. The
>    node now consumes the team's own types via
>    `src/fsae_supervisor/team_adapters.py`; topic names live in
>    `config/platforms/*.yaml`. See INTEGRATION_TODO.md.

Traceability audit between the MATLAB reference and the ROS port.
Each section lists the MATLAB source file, the Python module that
ports it, and any behavioural deltas.

## Design constraints the port had to preserve

1. **Deterministic mode ordering.** The 6-mode set, the coarse
   mapping `[1,2,2,2,2,3]`, and the `inflate_idx` lists are the
   public contract with `fault_dictionary.md`. They live in YAML now
   but the order is unchanged.
2. **Per-step budget.** The supervisor runs at 20 Hz. Each step has
   to complete in ≤5 ms — the MPC solver budgets 50 ms, the EKF
   update another 10–15 ms, and the supervisor steals what's left.
   The IMM inner loop is the hot spot; in the port it's vectorised
   with numpy over the validity-masked channel subset, so cost scales
   with `k_valid` rather than `n=12`.
3. **Exact math.** The log-domain Gaussian likelihood with Z_CLAMP=5
   is reproduced byte-for-byte. `test_imm_math.py` cross-checks the
   vectorised implementation against a scalar reference.
4. **No new failure modes.** The ROS port must not introduce
   behaviours that can't happen in the MATLAB version. In
   particular: message-drop robustness (freshness timers already
   handle this), thread safety (one mutex on the sensor caches),
   and no blocking waits in the hot path.

## File-by-file audit

### `supervisorConfig.m` → `config/supervisor.yaml` + `src/fsae_supervisor/config_loader.py`

The sigma-inflation math is reproduced exactly:

```
MATLAB:   sigma_j = sigma_nom .* (1 + (f - 1) * oneHot(idx))
Python:   sigma_j[i] = f * sigma_nom[i] if i in idx else sigma_nom[i]
```

`inflate_idx` stays 1-indexed in the YAML to match the MATLAB spec;
`config_loader.from_dict()` converts to 0-indexed once when it
builds the `(M, n)` sigma matrix.

### `computeResiduals.m` → `src/fsae_supervisor/residuals.py`

Every one of the 12 channels ported with identical semantics:

| Channel | MATLAB check → Python check |
|---|---|
| r(1) sigma_pos | `isfield(ekf,'P') && size(ekf.P,1) >= 2` → `ekf.P is not None and P.shape[0] >= 2` |
| r(2) cte | unconditional |
| r(3) speed_dev | `length(x_hat) >= 4` → `x.size >= 4` |
| r(4) NIS_mean | `isfinite(ekf.last_NIS)` → `np.isfinite` + ring buffer `>= 3` |
| r(5) mpc_time_norm | rolling mean / budget_ms |
| r(6) imu_enc_parity | `(v_enc / L) * tan(delta_cmd)` — `L` from config |
| r(7) steer_parity | QCar-specific: `steering_meas` if present, else binary surrogate |
| r(8) drive_parity | `abs(v_ref - v_meas)` as soft residual |
| r(9), r(10) cam/lid_age | timer increments each step, resets on fresh detection |
| r(11) imu_freeze | `max(0, log10(eps / var(omega)))` — unconditional |
| r(12) enc_freeze | same, gated by `|v_ref| > 0.1` |

Hard-override accumulators (`steer_err_time`, `drive_err_time`)
persist across steps and zero on the first healthy step — same as
`sup.actuator.steer_err_time` in MATLAB.

Minor delta: in MATLAB the sensor `valid` field defaults to `true`
if missing; in Python the dataclass defaults are explicit. Same
result, less magic.

### `imm.m` → `src/fsae_supervisor/imm.py`

Three-step structure identical:

1. Predict — `mu_bar = TPM' * mu`, floored, renormalised
2. Log-likelihood — diagonal Gaussian, Z_CLAMP=5, summed over valid channels, floored
3. Update — log-domain `log_post = loglik + log(mu_bar)`, subtract max, exponentiate, normalise, floor, renormalise

The only difference is that MATLAB loops over channels; Python
slices out the valid-channel columns and vectorises the z-score
and clip. Same floating-point result within `1e-9` (verified by the
`test_loglik_matches_reference` test).

### `actionPolicy.m` → `src/fsae_supervisor/action_policy.py`

Pipeline identical:

1. Hard-override latch — `override_latch`, `override_cause` persist on this
2. Coarse probability from `coarse_map`
3. Hysteresis gate on `k_above_critical`, `k_above_degraded`, `k_below_degraded`
4. `prev_mode` absorbing Critical
5. Argmax over Degraded sub-modes for isolation

One cosmetic tightening: `override_cause` is exposed on the action
message as a comma-separated string, so downstream logging can
record *why* the latch fired. The MATLAB version only stashed it
on `sup.override_cause` for printf purposes.

### `faultInjection.m` + `initFaultConfig.m` → `scripts/fault_injector_node.py` + `config/fault_injector.yaml`

The 8 presets (`none`, `camera_loss`, `lidar_loss`,
`both_sensors`, `imu_freeze`, `steering`, `cascade`,
`obstacle_on_track`) are ported to YAML blocks under
`fault_injector.presets.*`. The node applies the named preset at
startup; the `~set_fault` service allows runtime activation for
tests that don't want to restart the node.

Timing: MATLAB uses absolute simulation time `t`. ROS uses a
"time-since-arm" clock zeroed by the `~arm` service, so scripted
faults fire at the same relative time regardless of when the node
started. Either `~arm` is called explicitly, or the node
auto-arms at startup when `~auto_arm` is true (the default).

Fault types:

- **camera, lidar** — suppress the cone array (blank `ConeDetectionArray` / `PoseArray` republish). Injection is supervisor-side only: the team stack still reads real topics. Keeps the topic alive so a subscriber sees "empty frame" not "silent topic".
- **imu** — `freeze` snapshots `omega_z` on first firing step; `bias` adds a constant; `noise` adds scaled white noise.
- **encoder** — `freeze` / `zero` / `scale` same semantics as MATLAB.
- **steering, drive** — state is tracked, but not currently consumed (r7 disabled; no MpcStats). See INTEGRATION_TODO.md.
- **obstacle** — inject a synthetic 0.10 m return into the LaserScan centre beam so `min(ranges) < obstacle_min_dist`.

### Mission FSM → `scripts/mission_manager_node.py` + `config/mission.yaml` (original ROS code; no MATLAB source supplied)

FSM states and transitions identical. State codes (1=Waiting,
2=Running, 3=Finished, 4=EStopped) are exposed as constants on
`MissionStatus.msg`.

The keyboard interface (SPACE for go, ESC for estop — handled by
`onMissionKeyPress` in MATLAB) is replaced with the `SetMission`
service, which takes `{command: 'go' | 'estop' | 'reset'}`. The
estop transition is also driven automatically from the supervisor —
the mission manager subscribes to `/supervisor/action` and
transitions to EStopped whenever `action.estop` is true.

Lap detection mirrors the MATLAB algorithm: `wrap_high` =
round(0.85 * n_waypoints), `wrap_low` = round(0.15 * n_waypoints),
min_lap_time = 3.0 s debounce. The waypoint-wrap scheme needs a
waypoint-index topic that the team stack does not publish, so the
default `lap_source` is now `guidance_status` (laps_done from the
team guidance node's JSON status).

### `supervisorStep.m` → `scripts/supervisor_node.py`

This is the one MATLAB file whose structure doesn't survive the
port verbatim. In MATLAB, `supervisorStep` is called once per
simulation step from `run.m` with a bundle of arguments. In ROS,
the arguments arrive on different topics at different rates, so
the node caches the latest message per topic (guarded by a single
`threading.Lock`) and runs the same three-stage pipeline on a
fixed-period `rospy.Timer`.

Design choice: the supervisor runs on its own timer even when no
sensor messages have arrived. This is deliberate — the freshness
residuals r(9) and r(10) exist precisely to detect the topic-silence
condition, and the MATLAB semantics require them to grow at `dt`
whether or not new cones arrive. A "subscribe-and-block-on-IMU"
architecture would have made this impossible.

## Behavioural equivalence checklist

The items below are the MATLAB scenario set. **They are NOT yet
re-run on the port** (see correction 7 above); treat this as the
target validation list:

- 9 IMM detection scenarios (cam/lidar/IMU-freeze/IMU-bias/IMU-noise/enc-freeze/enc-dropout/EKF-divergence/MPC-overrun)
- 4 hard-override scenarios (cones-both-lost/steering/drive-stall/obstacle-imminent)
- 6 false-positive robustness scenarios (nominal baseline, noisy sensors, MPC transient spike, cornering lag, single-sensor flicker, soft drive lag)

`test/` currently has unit tests only; porting these scenarios is a TODO.

## Known gaps vs MATLAB

- **NIS activation.** The MATLAB note still applies: r(4) is inactive
  until the EKF populates `last_nis` on each update. The ROS port
  exposes `last_nis` as a field on `EkfState.msg`; whoever owns the
  EKF node has a 3-line patch to make.
- **Dashboard.** The MATLAB `plotSupervisorDashboard.m` doesn't have
  a direct port. The intent is that `rqt_plot` / `rosbag play`
  cover the same ground using the published posterior and residual
  topics. If a live dashboard is wanted, a separate visualisation
  node is straightforward — ~100 lines against the four supervisor
  topics.
- **Synthetic driver removed** in the integrated package (it published
  the old custom msg types). Use Gazebo + `fault_injector:=true`.
- **Scenario driver.** The MATLAB `run.m` does the entire simulation.
  The ROS port expects the sim (Gazebo) to drive the sensor topics
  and the control stack to publish `MpcStats` and `EkfState`.
  Nothing in this package owns the simulation timeline.
