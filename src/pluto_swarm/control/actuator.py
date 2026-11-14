"""ActuatorModel - what the firmware does with the sticks we sent, as seen from
the ground station. Feeds the estimator's prediction (localization/estimator.py)
so the filter knows the acceleration we commanded, delayed and lagged exactly
like the real drone (MagisV2 angle mode + alt-hold, config/plant/pluto_v5.yaml).

  stick -> tilt set-point (k deg/us, clamp max_tilt)  [after the link delay]
        -> first-order attitude lag (attitude_tau_s)
        -> body acceleration (g tan theta forward, -g tan phi left)
        -> rotated into the arena frame with the estimated yaw
  throttle -> effective climb-rate command (deadband applied), after the delay

The model uses the INTEGER stick values actually sent, so stick quantisation
(0.2 deg per us) is included.
"""

from __future__ import annotations

import collections
import math
from typing import Deque, Dict, Tuple

import numpy as np

from ..localization.estimator import Input


def stick_to_tilt(rc: int, k_deg_per_us: float, max_deg: float, sign: int) -> float:
    d = sign * (rc - 1500) * k_deg_per_us
    return math.radians(max(-max_deg, min(max_deg, d)))


def throttle_to_climb(rc: int, plant: Dict) -> float:
    d = rc - 1500
    if abs(d) <= plant["alt_hold_deadband_us"]:
        return 0.0
    v_cm = d * plant["alt_hold_cm_s_per_us"]
    v_cm = min(plant["alt_hold_v_max_cm_s"], max(plant["alt_hold_v_min_cm_s"], v_cm))
    return v_cm / 100.0


class ActuatorModel:
    def __init__(self, plant: Dict, control: Dict) -> None:
        self.plant = plant
        self.g = float(plant.get("g", 9.81))
        self.k = float(plant["stick_to_tilt_deg_per_us"])
        self.max_deg = float(plant["max_tilt_deg"])
        self.tau = float(plant["attitude_tau_s"])
        self.delay = float(control.get("model", {}).get("command_delay_s", 0.05))
        signs = control.get("rc_signs", {})
        self.s_roll = int(signs.get("roll", 1))
        self.s_pitch = int(signs.get("pitch", 1))
        self._q: Deque[Tuple[float, float, float, float]] = collections.deque()
        self._active = (0.0, 0.0, 0.0)          # theta_sp, phi_sp, vz_cmd now acting on the drone
        self.theta = 0.0                         # modelled actual tilt (nose down +)
        self.phi = 0.0                           # (right wing down +)
        self._t = None

    def push(self, t_sent: float, rc_roll: int, rc_pitch: int, rc_throttle: int,
             airborne: bool = True, vz_override: float = None) -> None:
        """vz_override: when the FIRMWARE flies the height (take-off, land, alt-hold
        at a remembered height) our throttle stick says nothing about the climb
        rate - pass the measured vertical speed (baro vario) instead."""
        th = stick_to_tilt(rc_pitch, self.k, self.max_deg, self.s_pitch) if airborne else 0.0
        ph = stick_to_tilt(rc_roll, self.k, self.max_deg, self.s_roll) if airborne else 0.0
        if vz_override is not None:
            self._q.append((t_sent, th, ph, float(vz_override)))   # already "now" - no delay
            return
        vz = throttle_to_climb(rc_throttle, self.plant) if airborne else 0.0
        self._q.append((t_sent + self.delay, th, ph, vz))

    def input_at(self, t: float, yaw: float) -> Input:
        while self._q and self._q[0][0] <= t:
            _, th, ph, vz = self._q.popleft()
            self._active = (th, ph, vz)
        th_sp, ph_sp, vz = self._active
        if self._t is not None and t > self._t:
            a = 1.0 - math.exp(-(t - self._t) / self.tau)
            self.theta += (th_sp - self.theta) * a
            self.phi += (ph_sp - self.phi) * a
        self._t = t
        ax_b = self.g * math.tan(self.theta)
        ay_b = -self.g * math.tan(self.phi)
        c, s = math.cos(yaw), math.sin(yaw)
        return Input(np.array([c * ax_b - s * ay_b, s * ax_b + c * ay_b]), vz)

    def reset(self) -> None:
        self._q.clear()
        self._active = (0.0, 0.0, 0.0)
        self.theta = self.phi = 0.0
