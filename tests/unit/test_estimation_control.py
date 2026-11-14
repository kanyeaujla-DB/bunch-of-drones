"""Kalman filter, actuator model, RC mapper, gain design."""

import math

import numpy as np
import pytest
import yaml

from pluto_swarm.control.actuator import ActuatorModel, throttle_to_climb
from pluto_swarm.control.design import design_pd
from pluto_swarm.control.position_controller import PositionController
from pluto_swarm.control.rc_mapper import RCMapper
from pluto_swarm.core.config import load_config, repo_root
from pluto_swarm.core.types import Setpoint
from pluto_swarm.localization.estimator import DroneEstimator, Input

ROOT = repo_root()
LOC = yaml.safe_load((ROOT / "config/localization.yaml").read_text())
CFG = load_config("sim", "hover_test")
R_CAM = np.diag([0.005 ** 2, 0.005 ** 2, 0.015 ** 2])


def _run_filter(order: str, seed: int = 0):
    """Same measurements, delivered in order or late/out of order -> same answer."""
    rng = np.random.default_rng(seed)
    e = DroneEstimator(LOC, 0.3)
    truth = np.array([1.0, 1.5, 1.0])
    pending = []
    for k in range(200):
        t = k * 0.02
        e.predict_to(t, Input(np.zeros(2), 0.0))
        if k % 3 == 0:
            pending.append(("cam", truth + rng.normal(0, [0.005, 0.005, 0.015]), t))
        if k % 2 == 0:
            pending.append(("baro", 0.4 + rng.normal(0, 0.03), t))
        delay = 0 if order == "inorder" else 3          # delivered 60 ms late, interleaved
        while pending and pending[0][2] <= t - delay * 0.02 + 1e-9:
            kind, z, tm = pending.pop(0)
            if kind == "cam":
                e.update_camera(z, R_CAM, tm)
            else:
                e.update_baro(z, tm)
    return e


def test_delayed_measurements_give_the_same_estimate():
    a, b = _run_filter("inorder"), _run_filter("late")
    assert np.allclose(a.pos, b.pos, atol=0.01)
    assert max(b.pos_std()) < 0.03                       # late baro must not erase camera info


def test_filter_learns_a_constant_drift():
    e = DroneEstimator(LOC, 0.3)
    bias = np.array([0.15, -0.1])
    p, v = np.array([1.0, 1.0, 1.0]), np.zeros(3)
    rng = np.random.default_rng(1)
    for k in range(1500):
        t = k * 0.02
        v[:2] += bias * 0.02
        p += v * 0.02
        e.predict_to(t, Input(np.zeros(2), 0.0))
        if k % 2 == 0:
            e.update_camera(p + rng.normal(0, 0.004, 3), R_CAM, t)
    assert np.allclose(e.bias[:2], bias, atol=0.03)


def test_rc_mapper_and_actuator_are_inverse():
    m = RCMapper(CFG.plant, CFG.control)
    act = ActuatorModel(CFG.plant, CFG.control)
    for yaw in (0.0, 0.7, -2.0):
        for a in ([0.5, 0.0], [0.0, -0.8], [0.6, 0.6]):
            roll, pitch = m.tilt_sticks(a[0], a[1], yaw)
            act.reset()
            act.push(0.0, roll, pitch, 1500)
            u = act.input_at(5.0, yaw)
            for _ in range(200):                          # let the attitude lag settle
                u = act.input_at(5.0 + _ * 0.01, yaw)
            assert np.allclose(u.a_xy, a, atol=0.03)


def test_dither_averages_to_small_climb_rates():
    m = RCMapper(CFG.plant, CFG.control)
    for target in (0.03, -0.06, 0.09):
        rates = [m.throttle(1, target)[1] for _ in range(400)]
        assert np.mean(rates) == pytest.approx(target, abs=0.004)
    rc, eff = m.throttle(2, 0.0)
    assert rc == 1500 and eff == 0.0
    assert throttle_to_climb(1530, CFG.plant) == 0.0     # inside the firmware deadband


def test_controller_cancels_drift_at_rest():
    c = PositionController(CFG.plant, CFG.control)
    sp = Setpoint(1, 0.0, np.array([1.0, 1.0, 1.0]))
    out = c.compute(sp.pos.copy(), np.zeros(3), np.array([0.2, -0.1, 0.0]), sp, 0.02)
    assert np.allclose(out.a_xy, [-0.2, 0.1])


@pytest.mark.parametrize("delay", [0.0, 0.04, 0.1])
def test_designed_gains_are_stable_and_damped(delay):
    kp, kd, w = design_pd(0.15, delay, 50)
    assert kp > 0.5 and kd > 0.5
    from pluto_swarm.control.design import closed_loop_poles
    assert np.max(np.abs(closed_loop_poles(kp, kd, 0.15, delay, 0.02))) < 1.0
