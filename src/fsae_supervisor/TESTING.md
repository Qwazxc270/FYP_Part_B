# Testing the supervisor without teammates' modules

> **Integration note:** `synthetic_test.launch` / `synthetic_driver_node.py` were removed in the integrated package. Use `pytest test/` (47 unit tests, no ROS needed), `catkin_make run_tests`, or Gazebo with `fault_injector:=true` (see INTEGRATION_TODO.md).

Three tiers, from fastest to most integrated. Work through them in
order — if tier N fails, don't bother running tier N+1.

## Tier 1 — Pure Python unit tests (no ROS)

Proves the algorithm is correct. Runs in under a second.

```bash
cd fsae_supervisor
PYTHONPATH=src python3 -m pytest test/ -v
```

Expected: 20 passed. Covers 19 scenarios + IMM math + action policy +
hysteresis. If this fails, nothing else will work.

## Tier 2 — ROS plumbing (no teammate modules)

Proves the supervisor works as a ROS node — topics, timers, services,
message schemas — without needing anyone else's code to be ready.

### One-time setup

```bash
cd ~/catkin_ws/src
ln -s /path/to/fsae_supervisor .
cd ..
catkin_make
source devel/setup.bash
```

### Liveness check

Bring up just the supervisor against no inputs. It should tick at 20 Hz
and eventually declare a cones_both_timeout because nothing is publishing.

```bash
roslaunch fsae_supervisor supervisor.launch
# in another terminal:
rostopic hz /supervisor/action              # expect 20.0 Hz
rostopic echo /supervisor/action            # watch mode drift to Critical
```

### End-to-end scenario tests with the synthetic driver

```bash
roslaunch fsae_supervisor synthetic_test.launch scenario:=nominal
roslaunch fsae_supervisor synthetic_test.launch scenario:=cam_dropout
roslaunch fsae_supervisor synthetic_test.launch scenario:=both_cones
roslaunch fsae_supervisor synthetic_test.launch scenario:=imu_freeze
roslaunch fsae_supervisor synthetic_test.launch scenario:=drive_stall
roslaunch fsae_supervisor synthetic_test.launch scenario:=obstacle
roslaunch fsae_supervisor synthetic_test.launch scenario:=cascade
```

Each run finishes with a summary line from the synthetic driver:

    final mode        : Degraded (Degraded-Perception)
    final speed_scale : 0.50
    estop             : False
    fault_isolation   : Perception
    override_cause    : ''

Expected final states:

| scenario | expected final mode | expected override_cause |
|---|---|---|
| nominal | Nominal | '' |
| cam_dropout | Degraded-Perception | '' |
| lidar_dropout | Degraded-Perception | '' |
| both_cones | Critical | cones_both_timeout |
| imu_freeze | Degraded-Proprioception | '' |
| drive_stall | Critical | drive_critical |
| obstacle | Critical | obstacle_imminent |
| cascade | Critical | obstacle_imminent |

These should line up 1-for-1 with the pytest scenarios in `test/test_residuals.py`.
If tier 1 passes but tier 2 doesn't match, the bug is in ROS-land
(message serialisation, topic names, timing) — not in the algorithm.

### Service interface smoke test

With `synthetic_test.launch` running a nominal scenario, exercise the
services from the command line:

```bash
# Mission state machine
rosservice call /mission_manager/set_mission "command: 'estop'"
rostopic echo -n1 /mission/status                # state should be 4

rosservice call /mission_manager/set_mission "command: 'reset'"
rostopic echo -n1 /mission/status                # state should be 1

# Runtime fault injection on top of the synthetic driver
rosservice call /fault_injector/set_fault "
subsystem: 'imu'
type: 'freeze'
value: 0.0
duration_s: 5.0
active: true"

# Watch the posterior shift
rostopic echo /supervisor/posterior | grep -A1 mu_fine
```

### Timing check

On the target CPU, confirm the 20 Hz timer actually holds and the
supervisor isn't missing ticks:

```bash
rostopic hz /supervisor/action                   # should be 20.0 ± 0.1
rostopic hz /supervisor/residuals                # same
```

If hz drifts below 19.5 Hz, the platform is too slow — reduce rate_hz
in supervisor.yaml and re-test.

## Tier 3 — Full stack (needs teammates' modules)

Semester 2. Replace the synthetic driver with the real sensor drivers,
EKF, MPC, and perception. The supervisor node and its YAML config
should need no changes — it consumes ROS topics, not Python modules,
so the only thing that matters is that the topic names and message
types agree.

Use the topic table in `supervisor.yaml` as the interface contract.
Any teammate who publishes those topics at reasonable rates is
supervisor-compatible by construction.

## Debugging tips

- **Supervisor stuck in Nominal when you expect Degraded?** Check that
  the synthetic driver is actually dropping the intended topic with
  `rostopic hz /perception/rgbd/cones` — if it's still publishing,
  the phase editor isn't firing.
- **Mode oscillates between Nominal and Degraded?** Hysteresis counters
  need more lead time; increase `k_enter_degraded` in supervisor.yaml.
- **Supervisor declares Critical immediately at launch?** Freshness
  residuals (r9, r10) fire before any sensor data has arrived.
  Either start the synthetic driver *before* the supervisor, or raise
  `override.cones_both_timeout` from 0.5 to something like 2.0 during
  bring-up.
- **Mission manager never exits Waiting?** You need to call
  `/mission_manager/set_mission 'go'`. The synthetic driver does this
  automatically after 2 s if `auto_go:=true` (the default in
  `synthetic_test.launch`).
