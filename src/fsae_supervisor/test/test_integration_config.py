"""Integration-config tests: platform profiles, channel mask, explicit
coarse mapping, and the latch=True subscriber regression."""
import copy, os, sys
import numpy as np, yaml
import pytest
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "src"))
from fsae_supervisor import (SupervisorConfig, ResidualGenerator, IMM, ActionPolicy,
                             SupervisorInputs, IMUData, EncoderData, ConeObs, EKFSnapshot)
from fsae_supervisor.config_loader import deep_merge

BASE = os.path.join(ROOT, "config", "supervisor.yaml")
PLAT = os.path.join(ROOT, "config", "platforms")
DISABLED = {"cte", "NIS_mean", "mpc_time_norm", "steer_parity", "filter_gap"}


def _check_profile(plat):
    cfg = SupervisorConfig.from_yaml(BASE, os.path.join(PLAT, plat + ".yaml"))
    assert cfg.platform.name == plat
    assert cfg.topics.action_out == "/supervisor/action"
    assert cfg.topics.odom_in == "/odometry/local"
    assert cfg.topics.odom_ref_in == "/odometry/filtered"


def test_profile_gazebo_loads():  _check_profile("gazebo")
def test_profile_qcar_hw_loads(): _check_profile("qcar_hw")


def test_gazebo_topics_are_team_topics():
    t = SupervisorConfig.from_yaml(BASE, os.path.join(PLAT, "gazebo.yaml")).topics
    assert t.imu_in == "/imu" and t.lidar_scan_in == "/scan"
    assert t.wheel_rl_vel_in == "/wheelrl_motor/velocity"
    assert t.cones_cam_in == "/cone_detections_camera"
    assert t.cones_lidar_in == "/cone_detections_lidar"
    assert t.mpc_stats_in == "" and t.ekf_state_in == ""


def test_fault_injection_overlay_redirects_inputs():
    cfg = SupervisorConfig.from_yaml(BASE, os.path.join(PLAT, "gazebo.yaml"),
                                     os.path.join(PLAT, "fault_injection_remap.yaml"))
    assert cfg.topics.imu_in == "/supervisor_fi/imu"
    assert cfg.topics.odom_in == "/odometry/local"   # untouched


def test_disabled_channels_never_valid():
    cfg = SupervisorConfig.from_yaml(BASE, os.path.join(PLAT, "gazebo.yaml"))
    names = cfg.residual.names
    assert {n for n, e in zip(names, cfg.channels_enabled) if not e} == DISABLED
    rg = ResidualGenerator(cfg)
    inp = SupervisorInputs(imu=IMUData(w=0.1), enc=EncoderData(v=0.5),
                           ekf=EKFSnapshot(x=np.array([0, 0, 0, 0.5]), P=np.eye(4) * 0.01, last_nis=2.0))
    inp.cte = 5.0; inp.mpc_time_ms = 500.0; inp.steering_meas = 1.0
    for _ in range(5):
        r, v, f = rg.step(inp, cfg.dt)
    for n in DISABLED:
        i = names.index(n)
        assert not v[i] and r[i] == 0.0
    assert not f.cte_exceeded and not f.mpc_overrun and not f.steering_fault
    assert v[names.index("sigma_pos")] and v[names.index("imu_enc_parity")]


def test_nominal_stays_nominal_with_mask():
    cfg = SupervisorConfig.from_yaml(BASE, os.path.join(PLAT, "gazebo.yaml"))
    rg, imm, pol = ResidualGenerator(cfg), IMM(cfg), ActionPolicy(cfg)
    rng = np.random.default_rng(0)
    for k in range(200):
        inp = SupervisorInputs(imu=IMUData(w=0.01 * rng.standard_normal()),
                               enc=EncoderData(v=0.7 + 0.02 * rng.standard_normal()),
                               cam_cones=ConeObs(np.array([1.0, 2.0])),
                               lidar_cones=ConeObs(np.array([1.5])),
                               ekf=EKFSnapshot(x=np.array([0, 0, 0, 0.7]), P=np.eye(4) * 0.01))
        inp.v_ref = 0.7
        r, v, f = rg.step(inp, cfg.dt)
        imm.step(r, v)
        act = pol.step(imm.mu, f)
    assert act.mode_coarse == 1 and act.mode_name == "Nominal"
    assert np.isclose(imm.coarse().sum(), 1.0)


def test_camera_and_lidar_loss_goes_critical_via_override():
    cfg = SupervisorConfig.from_yaml(BASE, os.path.join(PLAT, "gazebo.yaml"))
    rg, imm, pol = ResidualGenerator(cfg), IMM(cfg), ActionPolicy(cfg)
    for k in range(20):
        inp = SupervisorInputs(imu=IMUData(w=0.0), enc=EncoderData(v=0.7))
        r, v, f = rg.step(inp, cfg.dt); imm.step(r, v); act = pol.step(imm.mu, f)
    assert act.mode_coarse == 3 and act.estop and "cones_both_timeout" in act.override_cause


def _raw():
    with open(BASE) as fh:
        return yaml.safe_load(fh)["supervisor"]


def test_coarse_of_is_authoritative_and_checked():
    d = _raw()
    assert SupervisorConfig.from_dict(d).mode.coarse_map.tolist() == [1, 2, 2, 2, 2, 3]
    bad = copy.deepcopy(d); bad["mode"]["coarse_map"] = [1, 2, 2, 2, 3, 3]
    with pytest.raises(ValueError):
        SupervisorConfig.from_dict(bad)
    bad = copy.deepcopy(d); bad["mode"]["coarse_of"]["Degraded-Compute"] = "Critical"
    del bad["mode"]["coarse_map"]
    with pytest.raises(ValueError):
        SupervisorConfig.from_dict(bad)


def test_unknown_enabled_channel_rejected():
    d = _raw(); d["residual"]["enabled"]["bogus"] = True
    with pytest.raises(ValueError):
        SupervisorConfig.from_dict(d)


def test_deep_merge():
    assert deep_merge({"a": {"b": 1, "c": 2}}, {"a": {"c": 3}}) == {"a": {"b": 1, "c": 3}}


def test_no_latch_on_subscribers():
    import re
    for fn in [f for f in os.listdir(os.path.join(ROOT, "scripts")) if f.endswith(".py")]:
        src = open(os.path.join(ROOT, "scripts", fn)).read()
        for m in re.finditer(r"rospy\.Subscriber\([^)]*\)", src):
            assert "latch" not in m.group(0), fn


# ---------------- filter_gap (r13) ----------------
def _gz():
    return SupervisorConfig.from_yaml(BASE, os.path.join(PLAT, "gazebo.yaml"))


def test_filter_gap_channel_configured_and_off_by_default():
    cfg = _gz()
    i = cfg.channel_index("filter_gap")
    assert i == 12 and cfg.residual.n == 13 and not cfg.channel_enabled("filter_gap")
    # signature: inflated in Perception (2) and Proprioception (3) only, plus Critical
    sig = cfg.mode.sigma_per_mode[:, i] / cfg.residual.sigma_nom[i]
    assert np.allclose(sig, [1, cfg.mode.inf_deg, cfg.mode.inf_deg, 1, 1, cfg.mode.inf_crit])


def test_filter_gap_masked_when_disabled():
    cfg = _gz(); rg = ResidualGenerator(cfg)
    inp = SupervisorInputs(); inp.filter_gap = 5.0
    r, v, _ = rg.step(inp, cfg.dt)
    assert not v[12] and r[12] == 0.0


def _enabled_fg_cfg():
    d = deep_merge(_raw(), yaml.safe_load(open(os.path.join(PLAT, "gazebo.yaml")))["supervisor"])
    d["residual"]["enabled"]["filter_gap"] = True
    return SupervisorConfig.from_dict(d)


def test_filter_gap_passes_through_when_enabled():
    cfg = _enabled_fg_cfg(); rg = ResidualGenerator(cfg)
    inp = SupervisorInputs(); inp.filter_gap = 0.4
    r, v, _ = rg.step(inp, cfg.dt)
    assert v[12] and np.isclose(r[12], 0.4)
    inp.filter_gap = None
    r, v, _ = rg.step(inp, cfg.dt)
    assert not v[12]


def test_filter_gap_large_drives_degraded_not_nominal():
    cfg = _enabled_fg_cfg()
    rg, imm, pol = ResidualGenerator(cfg), IMM(cfg), ActionPolicy(cfg)
    for k in range(60):
        inp = SupervisorInputs(imu=IMUData(w=0.001 * ((-1) ** k)), enc=EncoderData(v=0.7 + 0.01 * ((-1) ** k)),
                               cam_cones=ConeObs(np.array([1.0])), lidar_cones=ConeObs(np.array([1.0])),
                               ekf=EKFSnapshot(x=np.array([0, 0, 0, 0.7]), P=np.eye(4) * 0.01))
        inp.v_ref = 0.7
        inp.filter_gap = 3.0 if k >= 20 else 0.1
        r, v, f = rg.step(inp, cfg.dt); imm.step(r, v); act = pol.step(imm.mu, f)
    assert act.mode_coarse == 2
    assert act.fault_isolation in ("Perception", "Proprioception")
