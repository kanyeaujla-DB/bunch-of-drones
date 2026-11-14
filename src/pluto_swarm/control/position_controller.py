"""Position controller (architecture doc §5) - one instance per drone.

Horizontal (per axis, arena frame):
    a = a_ref + c v_ref + Kp (p_ref - p) + Kd (v_ref - v) + Ki * int(e) - d_hat
        (c v_ref: drag feed-forward, c = plant drag_per_s - without it a moving
         reference is followed with a steady lag of c*v/Kp)
Vertical (alt-hold takes a climb-rate request with first-order lag tau_z):
    vz = vz_ref + tau_z * az_ref + Kz (z_ref - z) + Ki_z * int(e_z) - tau_z * d_hat_z
         ^ feed-forward    ^ lead for the alt-hold lag      ^ drift cancel

d_hat is the Kalman filter's drift estimate: cancelling it directly IS the
drift correction (the small integrators are only a backstop, with anti-windup).

The state passed in should be the filter state PREDICTED AHEAD by the command
delay (Smith-predictor style), so the loop acts on where the drone will be when
the command lands.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np

from ..core.types import Setpoint


@dataclass
class ControlOutput:
    a_xy: np.ndarray          # desired horizontal acceleration (arena), m/s^2
    vz: float                 # desired climb rate, m/s
    az: float                 # equivalent vertical acceleration (for the safety filter)


class PositionController:
    def __init__(self, plant: Dict, control: Dict) -> None:
        pos = control.get("position", {})
        self.kp = float(pos.get("kp", 1.2))
        self.kd = float(pos.get("kd", 1.6))
        self.ki = float(pos.get("ki", 0.0))
        self.kz = float(pos.get("z_kp", 1.0))
        self.kiz = float(pos.get("z_ki", 0.0))
        self.i_lim = float(pos.get("i_limit_m_s2", 0.3))
        self.g = float(plant.get("g", 9.81))
        self.a_max = self.g * np.tan(np.radians(min(float(pos.get("max_tilt_cmd_deg", 10.0)),
                                                    float(plant["max_tilt_deg"]))))
        self.tau_z = float(plant["z_tau_s"])
        self.vz_min = float(plant["alt_hold_v_min_cm_s"]) / 100.0 * 0.9
        self.vz_max = float(plant["alt_hold_v_max_cm_s"]) / 100.0 * 0.9
        self.use_bias = bool(pos.get("cancel_drift", True))
        self.drag = float(plant.get("drag_per_s", 0.0))
        self._i = np.zeros(3)

    def reset(self) -> None:
        self._i[:] = 0.0

    def compute(self, p: np.ndarray, v: np.ndarray, d_hat: np.ndarray, sp: Setpoint,
                dt: float, xy_enabled: bool = True) -> ControlOutput:
        e = sp.pos - p
        ev = sp.vel - v
        d = d_hat if self.use_bias else np.zeros(3)
        a = sp.acc[:2] + self.drag * sp.vel[:2] + self.kp * e[:2] + self.kd * ev[:2] + self._i[:2] - d[:2]
        n = float(np.linalg.norm(a))
        saturated = n > self.a_max
        if saturated:
            a *= self.a_max / n
        vz = sp.vel[2] + self.tau_z * sp.acc[2] + self.kz * e[2] + self._i[2] - self.tau_z * d[2]
        vz_sat = min(self.vz_max, max(self.vz_min, vz))
        # integrators with anti-windup: freeze while the output is saturated
        if self.ki > 0 and xy_enabled and not saturated:
            self._i[:2] = np.clip(self._i[:2] + self.ki * e[:2] * dt, -self.i_lim, self.i_lim)
        if self.kiz > 0 and vz == vz_sat:
            self._i[2] = float(np.clip(self._i[2] + self.kiz * e[2] * dt, -0.15, 0.15))
        az = (vz_sat - v[2]) / self.tau_z
        return ControlOutput(a if xy_enabled else np.zeros(2), vz_sat, az)

    def vz_from_az(self, az: float, vz_now: float) -> float:
        """Inverse of the alt-hold model: vertical acceleration -> climb-rate request."""
        return min(self.vz_max, max(self.vz_min, vz_now + self.tau_z * az))
