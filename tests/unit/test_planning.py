"""Formations, envelopes, transfers and slot assignment."""

import itertools
import math

import numpy as np
import pytest
import yaml

from pluto_swarm.core.config import repo_root
from pluto_swarm.core.hungarian import hungarian
from pluto_swarm.planning.assignment import assign_slots
from pluto_swarm.planning.envelopes import Envelope, smoothstep
from pluto_swarm.planning.formations.circle_wave import CircleWave
from pluto_swarm.planning.formations.line_wave import LineWave
from pluto_swarm.planning.min_jerk import MinJerk

ROOT = repo_root()
T1 = yaml.safe_load((ROOT / "config/missions/task1_line_wave.yaml").read_text())
T2 = yaml.safe_load((ROOT / "config/missions/task2_circle_wave.yaml").read_text())


def _num_deriv(f, tau, h=1e-4):
    return (f(tau + h) - f(tau - h)) / (2 * h)


@pytest.mark.parametrize("u", [0.0, 0.3, 0.5, 0.9, 1.0])
def test_smoothstep_values(u):
    s, ds, dds, integ = smoothstep(u)
    assert 0.0 <= s <= 1.0
    if u in (0.0, 1.0):
        assert ds == 0.0 and dds == 0.0


def test_envelope_integral_matches_numeric():
    env = Envelope(4.0, 6.0)
    taus = np.linspace(0, env.duration, 20001)
    vals = np.array([env.at(t)[0] for t in taus])
    numeric = np.concatenate([[0.0], np.cumsum((vals[1:] + vals[:-1]) / 2 * np.diff(taus))])
    for k in (3000, 9000, 15000, 20000):
        assert env.at(taus[k])[3] == pytest.approx(numeric[k], abs=1e-3)


@pytest.mark.parametrize("n", [2, 4, 7, 10])
def test_line_wave_any_n_slots_and_spacing(n):
    f = LineWave(list(range(1, n + 1)), {**T1, "spacing_m": 0.6})
    slots = f.slot_positions()
    xs = sorted(p[0] for p in slots)
    assert np.allclose(np.diff(xs), 0.6)
    assert np.mean(xs) == pytest.approx(T1["centre"][0])


def test_line_wave_is_vertical_with_90_degree_lag_and_crest_moving_plus_x():
    f = LineWave([1, 2, 3, 4], T1)
    tau = f.env.ramp_s + 1.0                        # full amplitude
    sp = f.setpoints(tau)
    for d in sp.values():                           # only Z moves
        assert d.vel[0] == 0 and d.vel[1] == 0
    phases = [math.atan2(sp[i].pos[2] - T1["height_m"], sp[i].vel[2] / f.omega) for i in (1, 2, 3, 4)]
    lag = [(phases[k] - phases[k + 1]) % (2 * math.pi) for k in range(3)]
    assert np.allclose(lag, math.pi / 2, atol=1e-6)   # each drone 90 deg behind the one on its left


def test_line_wave_feedforward_is_the_true_derivative():
    f = LineWave([1, 2, 3, 4], T1)
    for tau in (1.0, 7.0, 20.0, 33.0):
        sp = f.setpoints(tau)[3]
        v_num = _num_deriv(lambda t: f.setpoints(t)[3].pos, tau)
        a_num = _num_deriv(lambda t: f.setpoints(t)[3].vel, tau)
        assert np.allclose(sp.vel, v_num, atol=1e-5)
        assert np.allclose(sp.acc, a_num, atol=1e-4)


def test_circle_wave_geometry_direction_and_in_phase_z():
    f = CircleWave([1, 2, 3, 4], T2)
    tau = f.env.ramp_s + 2.0
    sp = f.setpoints(tau)
    c = np.array(T2["centre"])
    for d in sp.values():
        assert np.linalg.norm(d.pos[:2] - c) == pytest.approx(T2["radius_m"])
    z = [sp[i].pos[2] for i in (1, 2, 3, 4)]
    assert np.allclose(z, z[0])                     # m = 0: all bob together
    r = sp[1].pos[:2] - c
    assert r[0] * sp[1].vel[1] - r[1] * sp[1].vel[0] < 0     # clockwise seen from above
    chords = [np.linalg.norm(sp[a].pos[:2] - sp[b].pos[:2]) for a, b in ((1, 2), (2, 3), (3, 4), (4, 1))]
    assert np.allclose(chords, chords[0])


def test_circle_wave_feedforward_is_the_true_derivative():
    f = CircleWave([1, 2, 3, 4], T2)
    for tau in (2.0, 12.0, 25.0):
        sp = f.setpoints(tau)[2]
        assert np.allclose(sp.vel, _num_deriv(lambda t: f.setpoints(t)[2].pos, tau), atol=1e-5)
        assert np.allclose(sp.acc, _num_deriv(lambda t: f.setpoints(t)[2].vel, tau), atol=1e-4)


def test_formations_start_and_end_at_rest():
    for f in (LineWave([1, 2, 3, 4], T1), CircleWave([1, 2, 3, 4], T2)):
        for tau in (0.0, f.duration):
            for sp in f.setpoints(tau).values():
                assert np.allclose(sp.vel, 0, atol=1e-9) and np.allclose(sp.acc, 0, atol=1e-9)


def test_min_jerk_endpoints():
    mj = MinJerk([0, 0, 0], [1, 2, 0.5], 3.0)
    p, v, a = mj.at(0.0)
    assert np.allclose(p, 0) and np.allclose(v, 0)
    p, v, a = mj.at(3.0)
    assert np.allclose(p, [1, 2, 0.5]) and np.allclose(v, 0) and np.allclose(a, 0)


def test_hungarian_matches_brute_force():
    rng = np.random.default_rng(3)
    for n, m in ((4, 4), (5, 5), (3, 6), (6, 3)):
        C = rng.random((n, m))
        pairs = hungarian(C)
        cost = sum(C[r, c] for r, c in pairs)
        if n <= m:
            best = min(sum(C[i, p[i]] for i in range(n)) for p in itertools.permutations(range(m), n))
        else:
            best = min(sum(C[p[j], j] for j in range(m)) for p in itertools.permutations(range(n), m))
        assert cost == pytest.approx(best)


def test_assign_slots_never_crosses():
    start = {1: np.array([0.6, 1.2, 1.2]), 2: np.array([1.2, 1.2, 1.2]),
             3: np.array([1.8, 1.2, 1.2]), 4: np.array([2.4, 1.2, 1.2])}
    slots = LineWave([1, 2, 3, 4], T1).slot_positions()[::-1]
    a = assign_slots(start, slots)
    xs = [slots[a[i]][0] for i in (1, 2, 3, 4)]
    assert xs == sorted(xs)                          # order along the line is preserved
