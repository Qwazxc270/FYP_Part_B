#! /usr/bin/env python3
"""
offline_test.py -- run the full guidance stack on every track, no ROS needed.

    python3 extras/offline_test.py                 # all tracks
    python3 extras/offline_test.py Track1 Track3   # selected tracks
    python3 extras/offline_test.py --plot          # also save PNG plots

For each track: spawn at the real Gazebo start pose, drive the mapping lap
with local planning, detect lap completion, build the race line from the
SLAM-style map, then race. Reports lap times, cone hits, race-line metrics.

The car here is a kinematic bicycle with pure pursuit, NOT Luke's MPC and
NOT Gazebo physics. It checks the guidance logic end-to-end; it does not
prove driving performance on the real stack.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

import numpy as np                                   # noqa: E402
import guidance_core as gc                          # noqa: E402
import track_sim as ts                              # noqa: E402


def find_world(track):
    for base in (os.path.join(HERE, '..', '..', 'qcar_gazebo', 'worlds'),
                 os.path.join(HERE, '..', '..', '..', 'qcar_gazebo', 'worlds'),
                 os.path.join(HERE, '..', '..', '..'), '.'):
        p = os.path.join(base, track + '.world')
        if os.path.exists(p):
            return p
    raise FileNotFoundError(track + '.world')


def true_track_length(cones):
    """Length of the real circuit: midpoints of each blue cone and its nearest
    yellow cone, ordered by chaining along the blue boundary."""
    blue = [(x, y) for x, y, c in cones if c == ts.BLUE]
    yel = [(x, y) for x, y, c in cones if c == ts.YELLOW]
    chain, rest = [blue[0]], blue[1:]
    while rest:
        q = min(rest, key=lambda r: math.hypot(r[0] - chain[-1][0], r[1] - chain[-1][1]))
        chain.append(q)
        rest.remove(q)
    mids = []
    for b in chain:
        o = min(yel, key=lambda q: math.hypot(q[0] - b[0], q[1] - b[1]))
        mids.append(((b[0] + o[0]) / 2, (b[1] + o[1]) / 2))
    mids = np.array(mids + mids[:1])
    return float(np.sum(np.hypot(np.diff(mids[:, 0]), np.diff(mids[:, 1]))))


def run(track, laps=3, t_max=700.0, plot=False, verbose=False, params=None,
        seed=0, drift_deg=0.6, quiet=False):
    cones = ts.load_world_cones(find_world(track))
    spawn = ts.SPAWN[track]
    car = ts.SimCar(spawn)
    odo = ts.OdometryModel(spawn, drift_yaw_rate=math.radians(drift_deg), seed=seed)
    sens = ts.Sensors(cones, seed=seed + 100)

    logs = []

    def sink(level, msg):
        logs.append((level, msg))
        if not quiet and (verbose or (level != 'debug' and 'rejected detection' not in msg)):
            print("   [%s] %s" % (level, msg))

    prm = {'total_laps': laps}
    if params:
        prm.update(params)
    planner = gc.GuidancePlanner(prm, sink)

    dt, t = 0.02, 0.0
    local_hist = []              # (t, local_pose) for capture-time lookup
    next_det, next_map, next_plan = 0.0, 0.0, 0.0
    hits, hit_set = 0, set()
    true_path, modes = [], []
    max_dev = 0.0
    local_pose, slam_pose = spawn, spawn

    def local_at(stamp):
        best = min(local_hist, key=lambda h: abs(h[0] - stamp))
        return best[1]

    while t < t_max:
        local_pose, slam_pose = odo.update(car.pose, dt, t)
        local_hist.append((t, local_pose))
        if len(local_hist) > 600:
            local_hist.pop(0)
        planner.on_slam_pose(slam_pose, local_pose, t)

        if t >= next_det:                             # 10 Hz perception
            next_det = t + 0.1
            cam, lidar, _ = sens.detect(car.pose, t)
            cap = local_at(t - 0.0)
            planner.on_detections(cam, cap, t)
            planner.on_detections(lidar, cap, t)

        if t >= next_map:                             # 2 Hz SLAM map
            next_map = t + 0.5
            planner.on_map(sens.slam_map(), t)

        if t >= next_plan:                            # plan_rate_hz
            next_plan = t + 1.0 / planner.p['plan_rate_hz']
            out = planner.step(t, local_pose)
            st = out['status']
            modes.append((t, st['phase'], st['mode']))
            if st.get('deviation') is not None and not st.get('race_fallback'):
                max_dev = max(max_dev, st['deviation'])
            tr = out['traj']
            if tr is not None:
                # controller runs on LOCAL odometry: map its path to the true
                # frame through the exact local->true transform of this instant
                lx, ly, lyaw = local_pose
                tx, ty, tyaw = car.pose
                dyaw = ts.wrap(tyaw - lyaw)
                pts = []
                for x, y in zip(tr['x'], tr['y']):
                    rx, ry = gc.rot(x - lx, y - ly, dyaw)
                    pts.append((tx + rx, ty + ry))
                car.set_command(np.array(pts).T, tr['velocity'])
            if st['phase'] == 'FINISHED' and car.v < 0.02:
                break

        car.step(dt)
        true_path.append(car.pose)
        for i, (cx, cy, _) in enumerate(cones):
            if math.hypot(cx - car.x, cy - car.y) < 0.17:
                if i not in hit_set:
                    hit_set.add(i)
                    hits += 1
            elif i in hit_set and math.hypot(cx - car.x, cy - car.y) > 0.5:
                hit_set.discard(i)
        t += dt

    res = {
        'true_length': true_track_length(cones),
        'track': track, 'sim_time': t, 'laps': planner.laps.laps,
        'lap_times': planner.laps.lap_times, 'cone_hits': hits,
        'phase': planner.phase, 'race_built': planner.race is not None,
        'race_frame': planner.race_frame,
        'race_metrics': planner.race.metrics if planner.race else None,
        'max_race_deviation': max_dev,
        'rejected_frames': planner.memory.rejected_frames,
        'race_conflicts': planner.race_conflicts,
        'fallbacks': sum(1 for l, m in logs if 'falling back' in m),
        'recoveries': sum(1 for l, m in logs if 'RECOVERY' in m),
    }

    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 8))
        colmap = {ts.BLUE: 'tab:blue', ts.YELLOW: 'gold', ts.ORANGE: 'tab:orange'}
        for cx, cy, c in cones:
            ax.plot(cx, cy, 'o', color=colmap[c], ms=4)
        tp = np.array(true_path)
        ax.plot(tp[:, 0], tp[:, 1], '-', color='0.3', lw=0.8, label='driven path')
        if planner.race is not None:
            rl = np.vstack([planner.race.xy, planner.race.xy[:1]])
            ax.plot(rl[:, 0], rl[:, 1], '-', color='tab:green', lw=2, label='race line')
        ax.plot(*spawn[:2], 'k*', ms=12, label='start')
        ax.set_aspect('equal')
        ax.legend(loc='best')
        ax.set_title('%s: %d laps, %d cone hits' % (track, res['laps'], hits))
        out = os.path.join(HERE, 'offline_%s.png' % track)
        fig.savefig(out, dpi=110, bbox_inches='tight')
        plt.close(fig)
        res['plot'] = out
    return res


def summarise(r):
    print("\n=== %s ===" % r['track'])
    print("  laps done       : %d  (phase %s)" % (r['laps'], r['phase']))
    print("  lap times (s)   : %s" % ', '.join('%.1f' % x for x in r['lap_times']))
    print("  race line built : %s (%s frame)" % (r['race_built'], r['race_frame']))
    if r['race_metrics']:
        ratio = r['race_metrics']['lap_length_m'] / r['true_length']
        print("  CIRCUIT CHECK   : race lap %.1f m vs real track %.1f m (%.0f%%) %s"
              % (r['race_metrics']['lap_length_m'], r['true_length'], 100 * ratio,
                 "OK" if 0.85 <= ratio <= 1.1 else "<-- WRONG CIRCUIT"))
    m = r['race_metrics']
    if m:
        print("  length          : %.1f m (centreline %.1f m)" % (m['lap_length_m'], m['centreline_length_m']))
        print("  curvature energy: %.2f (centreline %.2f)  peak %.2f (centreline %.2f)"
              % (m['curvature_energy'], m['centreline_curvature_energy'],
                 m['peak_curvature'], m['centreline_peak_curvature']))
        print("  est. lap time   : %.1f s (centreline %.1f s)" % (m['est_lap_time_s'], m['centreline_est_lap_time_s']))
        print("  clearance       : %.2f m (margin %.2f)" % (m['min_clearance_m'], m['margin_used_m']))
    print("  max race dev    : %.2f m   fallbacks %d   recoveries %d"
          % (r['max_race_deviation'], r['fallbacks'], r['recoveries']))
    print("  cone hits       : %d   rejected phantom frames %d"
          % (r['cone_hits'], r['rejected_frames']))


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    tracks = args or ['Track1', 'Track2', 'Track3', 'CircleTrack', 'Figure8']
    results = []
    for tr in tracks:
        print("\n##### %s" % tr)
        results.append(run(tr, plot='--plot' in sys.argv, verbose='--verbose' in sys.argv))
    for r in results:
        summarise(r)
    ok = all(r['race_built'] and r['laps'] >= 3 and r['cone_hits'] == 0 and
             0.85 <= r['race_metrics']['lap_length_m'] / r['true_length'] <= 1.1
             for r in results)
    print("\nOVERALL: %s" % ("PASS" if ok else "CHECK RESULTS ABOVE"))
    sys.exit(0 if ok else 1)
