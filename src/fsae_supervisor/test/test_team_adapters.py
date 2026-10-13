"""Unit tests for team_adapters (no ROS needed)."""
import math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "src")))
from fsae_supervisor import team_adapters as ta


def test_wheel_speed_matches_nav_filter_formula():
    # nav_filter_node: v = r * 0.5 * (-w_rl + w_rr)
    v = ta.wheel_speed(-20.0, 20.0, 0.0325, -1.0, 1.0)
    assert math.isclose(v, 0.0325 * 0.5 * (20.0 + 20.0))

def test_wheel_speed_single_wheel_and_none():
    assert math.isclose(ta.wheel_speed(None, 10.0, 0.0325), 0.325)
    assert ta.wheel_speed(None, None, 0.0325) is None

def test_steering_cmd_mean():
    assert math.isclose(ta.steering_cmd(0.2, 0.1), 0.15)
    assert ta.steering_cmd(None, None) is None

def test_yaw_from_quaternion():
    assert math.isclose(ta.yaw_from_quaternion(0, 0, math.sin(0.5), math.cos(0.5)), 1.0, abs_tol=1e-9)

def test_odometry_to_ekf_cov_block():
    cov = np.zeros(36); cov[0] = 0.04; cov[7] = 0.09; cov[35] = 0.01; cov[1] = cov[6] = 0.001
    x, P = ta.odometry_to_ekf(1, 2, 0.3, 0.5, cov)
    assert np.allclose(x, [1, 2, 0.3, 0.5])
    assert P.shape == (4, 4) and P[0, 0] == 0.04 and P[1, 1] == 0.09 and P[2, 2] == 0.01 and P[0, 1] == 0.001

def test_odometry_zero_cov_gives_none():
    _, P = ta.odometry_to_ekf(0, 0, 0, 0, [0.0] * 36)
    assert P is None

def test_cone_ranges():
    assert np.allclose(ta.cone_ranges_from_points([3, 0], [4, 1]), [5, 1])
    assert ta.cone_ranges_from_points([], []).size == 0
    assert np.allclose(ta.cone_ranges_from_detections([0.0, 2.0], [3, 9], [4, 9]), [5, 2])

def test_scan_filter():
    r = ta.finite_scan_ranges([float("inf"), 0.01, 1.0, float("nan")], 0.05)
    assert np.allclose(r, [1.0])


# ---------------- filter_gap ----------------
def _drive(h, t0, t1, v, yaw_rate, x0=0.0, y0=0.0, yaw0=0.0, dt=0.05, offset=(0, 0, 0)):
    """Integrate a unicycle and push poses, optionally in a frame offset by (ox, oy, oyaw)."""
    x, y, yaw, t = x0, y0, yaw0, t0
    ox, oy, oh = offset
    while t <= t1 + 1e-9:
        c, s_ = math.cos(oh), math.sin(oh)
        h.push(t, ox + c * x - s_ * y, oy + s_ * x + c * y, ta._wrap(yaw + oh))
        x += v * math.cos(yaw) * dt; y += v * math.sin(yaw) * dt; yaw += yaw_rate * dt; t += dt


def test_filter_gap_zero_for_consistent_estimators_with_frame_offset():
    a, b = ta.PoseHistory(3), ta.PoseHistory(3)
    _drive(a, 0, 3, 0.7, 0.3)
    _drive(b, 0, 3, 0.7, 0.3, offset=(2.0, -1.0, 0.8))   # SLAM frame offset: NOT a fault
    r, dp, dh = ta.filter_gap(a, b, 1.0, 0.15, 0.1)
    assert r < 1e-3 and dp < 1e-3 and abs(dh) < 1e-6


def test_filter_gap_detects_scale_and_yaw_drift():
    a, b = ta.PoseHistory(3), ta.PoseHistory(3)
    _drive(a, 0, 3, 0.7, 0.0)
    _drive(b, 0, 3, 0.7 * 0.6, 0.2)      # wheel slip + heading drift
    r, dp, dh = ta.filter_gap(a, b, 1.0, 0.15, 0.1)
    assert dp > 0.2 and abs(dh) > 0.15 and r > 1.0


def test_filter_gap_time_alignment_uses_common_stamps():
    a, b = ta.PoseHistory(3), ta.PoseHistory(3)
    _drive(a, 0, 3.0, 0.7, 0.0)
    _drive(b, 0.013, 2.6, 0.7, 0.0, dt=0.1)     # different rate, phase and latency
    r, dp, _ = ta.filter_gap(a, b, 1.0, 0.15, 0.1)
    assert dp < 1e-6 and r < 1e-3


def test_filter_gap_none_when_insufficient_or_stale():
    a, b = ta.PoseHistory(3), ta.PoseHistory(3)
    assert ta.filter_gap(a, b, 1.0, 0.15, 0.1) is None
    _drive(a, 0, 0.5, 0.7, 0.0); _drive(b, 0, 0.5, 0.7, 0.0)
    assert ta.filter_gap(a, b, 1.0, 0.15, 0.1) is None           # window not covered
    _drive(a, 0.55, 2.0, 0.7, 0.0); _drive(b, 0.55, 2.0, 0.7, 0.0)
    assert ta.filter_gap(a, b, 1.0, 0.15, 0.1, t_now=5.0, max_age_s=0.5) is None   # stale
