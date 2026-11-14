"""The emulator must behave like the MagisV2 firmware facts in config/plant."""

import math

import numpy as np
import pytest

from pluto_swarm.core.config import load_config
from sim.plant.pluto_firmware_model import Phase, PlutoFirmwareModel

DT = 0.005


@pytest.fixture
def plant():
    p = dict(load_config("sim").plant)
    p["accel_bias_sigma_m_s2"] = 0.0
    p["baro_noise_cm"] = 0.0
    p["baro_drift_cm_per_min"] = 0.0
    return p


def fly(m, seconds, rc):
    for _ in range(int(seconds / DT)):
        m.on_rc(rc)
        m.step(DT)


def took_off(plant):
    m = PlutoFirmwareModel(plant, rng=np.random.default_rng(0))
    m.on_rc([1500, 1500, 1800, 1500, 1500, 1000, 1500, 1500])
    m.on_command(1)
    fly(m, 4.0, [1500, 1500, 1500, 1500, 1500, 1000, 1500, 1500])
    return m


def test_switch_alone_does_not_arm_with_high_throttle(plant):
    m = PlutoFirmwareModel(plant)
    m.on_rc([1500, 1500, 1800, 1500, 1500, 1000, 1500, 1500])
    assert m.phase == Phase.DISARMED


def test_takeoff_reaches_120cm(plant):
    m = took_off(plant)
    assert m.phase == Phase.FLYING
    assert m.s.pos[2] == pytest.approx(1.20, abs=0.05)


def test_deadband_ignores_small_throttle(plant):
    m = took_off(plant)
    z0 = m.s.pos[2]
    fly(m, 2.0, [1500, 1500, 1530, 1500, 1500, 1000, 1500, 1500])   # inside +-40
    assert m.s.pos[2] == pytest.approx(z0, abs=0.01)
    fly(m, 2.0, [1500, 1500, 1600, 1500, 1500, 1000, 1500, 1500])   # 100/4 = 25 cm/s
    assert m.s.vel[2] == pytest.approx(0.25, abs=0.02)


def test_tilt_saturates_at_20_deg(plant):
    m = took_off(plant)
    fly(m, 1.0, [1800, 1500, 1500, 1500, 1500, 1000, 1500, 1500])
    assert math.degrees(m.s.phi) == pytest.approx(20.0, abs=0.5)


def test_pitch_forward_moves_plus_x_at_zero_yaw(plant):
    m = took_off(plant)
    x0 = m.s.pos[0]
    fly(m, 1.0, [1500, 1550, 1500, 1500, 1500, 1000, 1500, 1500])
    assert m.s.pos[0] > x0 + 0.05


def test_rc_loss_triggers_land(plant):
    m = took_off(plant)
    for _ in range(int(6.0 / DT)):     # no on_rc calls -> stream lost
        m.step(DT)
    assert m.phase == Phase.DISARMED and m.s.pos[2] == 0.0


def test_land_then_needs_switch_toggle_to_rearm(plant):
    m = took_off(plant)
    m.on_command(2)
    fly(m, 6.0, [1500, 1500, 1000, 1500, 1500, 1000, 1500, 1500])
    assert m.phase == Phase.DISARMED
    m.on_rc([1500, 1500, 1000, 1500, 1500, 1000, 1500, 1500])
    assert m.phase == Phase.DISARMED           # switch still on: no re-arm
