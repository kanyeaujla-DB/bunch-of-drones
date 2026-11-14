"""Task 2 - rotating circle with Z oscillation (architecture doc §6).

    theta_i(tau) = 2 pi i / N + sgn * Omega * E(tau)       (E = integral of the envelope)
    (x_i, y_i)   = centre + R (cos theta_i, sin theta_i)
    z_i(tau)     = h + B e(tau) sin(w_z tau + m * 2 pi i / N)

sgn = -1 for clockwise seen from above (Drona's animation), +1 counter-clockwise.
m = 0: all drones bob in phase (default, as animated); m = 1: wave around the ring.
Rotation speed ramps with the same envelope, so the circle starts from rest.
"""

from __future__ import annotations

import math
from typing import Dict, List

import numpy as np

from ...core.types import Setpoint
from ..envelopes import Envelope
from .base import Formation


class CircleWave(Formation):
    def __init__(self, drone_ids: List[int], params: Dict, slot_of: Dict[int, int] = None) -> None:
        super().__init__(drone_ids, params)
        p = params
        self.n = len(drone_ids)
        self.slot_of = slot_of or {d: i for i, d in enumerate(drone_ids)}
        self.R = float(p["radius_m"])
        self.h = float(p["height_m"])
        self.B = float(p["z_amplitude_m"])
        self.Tz = float(p["z_period_s"])
        self.wz = 2 * math.pi / self.Tz
        self.Om = 2 * math.pi / float(p["rotation_period_s"])
        self.sgn = -1.0 if str(p.get("rotation_direction", "cw")).lower() == "cw" else 1.0
        self.m = int(p.get("m", 0))
        self.theta0 = math.radians(float(p.get("start_angle_deg", 90.0)))
        self.centre = np.array([float(c) for c in p.get("centre", [1.5, 1.5])])
        cycles = float(p.get("cycles", 4))
        self.env = Envelope(float(p.get("ramp_periods", 2)) * self.Tz, cycles * self.Tz)

    @property
    def duration(self) -> float:
        return self.env.duration

    def _theta(self, i: int, E: float) -> float:
        return self.theta0 + 2 * math.pi * i / self.n + self.sgn * self.Om * E

    def slot_positions(self) -> List[np.ndarray]:
        return [np.array([*(self.centre + self.R * np.array([math.cos(self._theta(i, 0.0)),
                                                             math.sin(self._theta(i, 0.0))])),
                          self.h]) for i in range(self.n)]

    def setpoints(self, tau: float) -> Dict[int, Setpoint]:
        e, de, dde, E = self.env.at(tau)
        thd = self.sgn * self.Om * e            # angular rate
        thdd = self.sgn * self.Om * de
        out = {}
        for d in self.ids:
            i = self.slot_of[d]
            th = self._theta(i, E)
            c, s = math.cos(th), math.sin(th)
            ph = self.wz * tau + self.m * 2 * math.pi * i / self.n
            sn, cs = math.sin(ph), math.cos(ph)
            pos = np.array([self.centre[0] + self.R * c, self.centre[1] + self.R * s,
                            self.h + self.B * e * sn])
            vel = np.array([-self.R * s * thd, self.R * c * thd,
                            self.B * (de * sn + e * self.wz * cs)])
            acc = np.array([self.R * (-s * thdd - c * thd ** 2), self.R * (c * thdd - s * thd ** 2),
                            self.B * (dde * sn + 2 * de * self.wz * cs - e * self.wz ** 2 * sn)])
            out[d] = Setpoint(d, tau, pos, vel, acc, 0.0)
        return out
