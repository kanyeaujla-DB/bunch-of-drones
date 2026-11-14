"""RC mapper - desired accelerations / climb rate / heading -> integer sticks.

Horizontal: world acceleration -> body frame (camera yaw) -> tilt angles
    theta (nose down, forward)   = atan( a_body_x / g)
    phi   (right wing down)      = atan(-a_body_y / g)
    stick = 1500 + sign * angle_deg / k            (k = 0.2 deg/us, unverified)

Vertical, and the deadband trick (architecture doc §5): the firmware ignores
throttle within +-40 us of 1500, so it can't be asked for climb rates below
~10 cm/s. Small corrections would vanish and the Z wave would stair-step.
A first-order sigma-delta modulator sends +-v_step for a fraction |v|/v_step
of the ticks; the alt-hold loop averages it into the small rate we wanted.

Signs (`rc_signs` in control.yaml) are the first thing to verify at Airlink 1.
"""

from __future__ import annotations

import math
from typing import Dict, Tuple

from .actuator import throttle_to_climb


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class RCMapper:
    def __init__(self, plant: Dict, control: Dict) -> None:
        self.plant = plant
        self.g = float(plant.get("g", 9.81))
        self.k = float(plant["stick_to_tilt_deg_per_us"])
        pos = control.get("position", {})
        self.max_tilt_deg = min(float(pos.get("max_tilt_cmd_deg", 10.0)), float(plant["max_tilt_deg"]))
        signs = control.get("rc_signs", {})
        self.s_roll = int(signs.get("roll", 1))
        self.s_pitch = int(signs.get("pitch", 1))
        self.s_yaw = int(signs.get("yaw", 1))
        db = int(plant["alt_hold_deadband_us"])
        self.db_us = db
        self.step_us = db + int(control.get("dither", {}).get("margin_us", 4))
        self.k_alt = float(plant["alt_hold_cm_s_per_us"]) / 100.0          # m/s per us
        self.v_step = self.step_us * self.k_alt
        self.dither_on = bool(control.get("dither", {}).get("enabled", True))
        yaw = control.get("yaw", {})
        self.k_yaw = float(yaw.get("kp", 1.0))
        self.max_yaw_rate = math.radians(float(yaw.get("max_rate_deg_s", 30.0)))
        self.yaw_k = float(plant.get("yaw_rate_deg_s_per_us", 0.4))
        self._acc: Dict[int, float] = {}

    # ---- horizontal --------------------------------------------------------
    def tilt_sticks(self, ax: float, ay: float, yaw: float) -> Tuple[int, int]:
        c, s = math.cos(yaw), math.sin(yaw)
        abx = c * ax + s * ay
        aby = -s * ax + c * ay
        theta = math.degrees(math.atan2(abx, self.g))
        phi = math.degrees(math.atan2(-aby, self.g))
        lim = self.max_tilt_deg
        theta = max(-lim, min(lim, theta))
        phi = max(-lim, min(lim, phi))
        rc_pitch = 1500 + int(round(self.s_pitch * theta / self.k))
        rc_roll = 1500 + int(round(self.s_roll * phi / self.k))
        return rc_roll, rc_pitch

    # ---- vertical ----------------------------------------------------------
    def throttle(self, drone_id: int, vz: float) -> Tuple[int, float]:
        """Climb-rate request (m/s) -> (stick, effective climb rate the firmware sees)."""
        if abs(vz) >= self.v_step or not self.dither_on:
            us = int(round(vz / self.k_alt))
            if 0 < abs(us) <= self.db_us and not self.dither_on:
                us = 0
            rc = 1500 + us
            self._acc[drone_id] = 0.0
            return rc, throttle_to_climb(rc, self.plant)
        acc = self._acc.get(drone_id, 0.0) + vz
        if acc >= self.v_step / 2:
            us, out = self.step_us, self.v_step
        elif acc <= -self.v_step / 2:
            us, out = -self.step_us, -self.v_step
        else:
            us, out = 0, 0.0
        self._acc[drone_id] = max(-self.v_step, min(self.v_step, acc - out))
        rc = 1500 + us
        return rc, throttle_to_climb(rc, self.plant)

    def reset(self, drone_id: int) -> None:
        self._acc[drone_id] = 0.0

    # ---- heading -----------------------------------------------------------
    def yaw_stick(self, yaw_ref: float, yaw: float) -> int:
        rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.k_yaw * wrap(yaw_ref - yaw)))
        return 1500 + int(round(self.s_yaw * math.degrees(rate) / self.yaw_k))
