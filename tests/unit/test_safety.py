"""Collision filter, geofence, plan validation, watchdog."""

import numpy as np
import pytest
import yaml

from pluto_swarm.core.config import load_config, repo_root
from pluto_swarm.core.types import Health, Telemetry
from pluto_swarm.planning.mission import MissionPlan
from pluto_swarm.safety.cbf_filter import SafetyFilter
from pluto_swarm.safety.geofence import Geofence
from pluto_swarm.safety.preflight_validation import validate_plan
from pluto_swarm.safety.watchdog import Verdict, Watchdog

CFG = load_config("sim", "task1_line_wave")
LIMITS = {"v_max": 0.4, "a_max": 0.8, "t_min": 3.0}


def _start(n=4):
    return {i: np.array([0.6 * i, 1.2, 1.2]) for i in range(1, n + 1)}


def test_cbf_prevents_head_on_collision():
    f = SafetyFilter(CFG.safety)
    p = {1: np.array([1.0, 1.5, 1.0]), 2: np.array([2.0, 1.5, 1.0])}
    v = {1: np.zeros(3), 2: np.zeros(3)}
    dt, closest = 0.02, 9.0
    for _ in range(400):                       # nominal controllers drive them INTO each other
        u_hat = {1: np.array([1.5, 0, 0]), 2: np.array([-1.5, 0, 0])}
        u = f.filter(p, v, u_hat)
        for i in (1, 2):
            v[i] = v[i] + u[i] * dt
            p[i] = p[i] + v[i] * dt
        closest = min(closest, float(np.linalg.norm(p[1] - p[2])))
    assert closest >= f.D * 0.97
    assert f.interventions > 0


def test_cbf_is_a_no_op_when_far_apart():
    f = SafetyFilter(CFG.safety)
    p = {1: np.array([0.5, 0.5, 1.0]), 2: np.array([2.5, 2.5, 1.0])}
    v = {1: np.zeros(3), 2: np.zeros(3)}
    u_hat = {1: np.array([0.3, -0.2, 0.1]), 2: np.array([-0.1, 0.0, 0.0])}
    u = f.filter(p, v, u_hat)
    assert all(np.allclose(u[i], u_hat[i]) for i in (1, 2))


def test_geofence():
    g = Geofence(CFG.arena)
    assert g.inside_soft([1.5, 1.5, 1.0])
    assert not g.inside_soft([0.1, 1.5, 1.0])
    assert np.allclose(g.clip([0.0, 3.0, 5.0])[:2], [0.3, 2.7])
    assert g.breached_hard([0.05, 1.5, 1.0])


@pytest.mark.parametrize("mission", ["task1_line_wave", "task2_circle_wave"])
def test_both_tasks_pass_plan_validation(mission):
    cfg = load_config("sim", mission)
    rep = validate_plan(MissionPlan(cfg.mission, _start(), LIMITS), cfg.arena, cfg.safety, cfg.plant, cfg.control)
    assert rep.ok, rep.problems
    assert rep.min_separation_m >= cfg.safety["min_separation_m"]


def test_too_tight_spacing_is_rejected():
    cfg = load_config("sim", "task1_line_wave")
    mission = {**cfg.mission, "spacing_m": 0.4}
    rep = validate_plan(MissionPlan(mission, _start(), LIMITS), cfg.arena, cfg.safety, cfg.plant, cfg.control)
    assert not rep.ok and any("separation" in p for p in rep.problems)


def test_ten_drone_line_does_not_fit_but_ten_drone_circle_does():
    cfg = load_config("sim", "task1_line_wave")
    start = {i: np.array([0.3 + 0.27 * i, 0.8 + 0.5 * (i % 2), 1.2]) for i in range(10)}
    rep = validate_plan(MissionPlan(cfg.mission, start, LIMITS), cfg.arena, cfg.safety, cfg.plant, cfg.control)
    assert not rep.ok                                   # 5.4 m line in a 3 m arena
    c2 = load_config("sim", "task2_circle_wave")
    circle = {**c2.mission, "radius_m": 1.0}
    start = {i: np.array([0.5 + (i % 5) * 0.5, 1.0 + (i // 5) * 1.0, 1.2]) for i in range(10)}
    rep = validate_plan(MissionPlan(circle, start, LIMITS), c2.arena, c2.safety, c2.plant, c2.control)
    assert rep.min_separation_m >= c2.safety["min_separation_m"]


class _Link:
    def __init__(self, age=0.0, v=4.0, ok=True):
        self.telemetry = Telemetry(t=100.0 - age, battery_v=v)
        self.connected = ok

    def telemetry_age(self, t):
        return t - self.telemetry.t


def test_watchdog_verdicts():
    w = Watchdog(CFG.safety, CFG.arena)
    p = np.array([1.5, 1.5, 1.0])
    assert w.check(100.0, 1, Health.OK, _Link(), p, p, True).verdict == Verdict.CONTINUE
    assert w.check(100.0, 1, Health.DEGRADED, _Link(), p, p, True).verdict == Verdict.HOLD
    assert w.check(100.0, 1, Health.LOST, _Link(), p, p, True).verdict == Verdict.LAND
    assert w.check(100.0, 1, Health.OK, _Link(age=2.0), p, p, True).verdict == Verdict.LAND
    assert w.check(100.0, 1, Health.OK, _Link(v=3.2), p, p, True).verdict == Verdict.LAND
    far = p + np.array([0.6, 0, 0])
    assert w.check(100.0, 2, Health.OK, _Link(), p, far, True).verdict == Verdict.CONTINUE
    assert w.check(101.5, 2, Health.OK, _Link(age=-1.5), p, far, True).verdict == Verdict.LAND   # diverged
