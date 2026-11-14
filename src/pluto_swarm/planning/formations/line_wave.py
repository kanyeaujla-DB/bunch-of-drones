"""Task 1 - line formation with a travelling wave (architecture doc §6).

Drones sit at fixed slots along `line_axis`; each oscillates along `wave_axis`
(default z: confirmed from Drona's animation) with a phase lag proportional to
its position, so the crest travels along +line_axis (direction = +1):

    s_i   = (i - (N-1)/2) * spacing
    w_i   = A e(tau) sin(w tau - direction * k * s_i),   k = 2 pi / wavelength

Position, velocity AND acceleration are returned (feed-forward keeps the wave
in phase; feedback only cleans up). Works for any N.
"""

from __future__ import annotations

import math
from typing import Dict, List

import numpy as np

from ...core.types import Setpoint
from ..envelopes import Envelope
from .base import Formation

AXES = {"x": 0, "y": 1, "z": 2}


class LineWave(Formation):
    def __init__(self, drone_ids: List[int], params: Dict, slot_of: Dict[int, int] = None) -> None:
        super().__init__(drone_ids, params)
        p = params
        self.n = len(drone_ids)
        self.slot_of = slot_of or {d: i for i, d in enumerate(drone_ids)}
        self.line = AXES[p.get("line_axis", "x")]
        self.wave = AXES[p.get("wave_axis", "z")]
        if self.line == self.wave or self.line == 2:
            raise ValueError("line_axis must be x or y and differ from wave_axis")
        self.dir = 1.0 if int(p.get("direction", 1)) >= 0 else -1.0
        self.spacing = float(p["spacing_m"])
        self.h = float(p["height_m"])
        self.A = float(p["amplitude_m"])
        self.T = float(p["period_s"])
        self.omega = 2 * math.pi / self.T
        lam = p.get("wavelength_m")
        self.lam = float(lam) if lam else self.n * self.spacing
        self.k = 2 * math.pi / self.lam
        self.centre = np.array([float(c) for c in p.get("centre", [1.5, 1.5])] + [self.h])
        cycles = float(p.get("cycles", 3))
        self.env = Envelope(float(p.get("ramp_periods", 2)) * self.T, cycles * self.T)

    @property
    def duration(self) -> float:
        return self.env.duration

    def slot_positions(self) -> List[np.ndarray]:
        out = []
        for i in range(self.n):
            p = self.centre.copy()
            p[self.line] += (i - (self.n - 1) / 2.0) * self.spacing
            out.append(p)
        return out

    def setpoints(self, tau: float) -> Dict[int, Setpoint]:
        e, de, dde, _ = self.env.at(tau)
        slots = self.slot_positions()
        out = {}
        for d in self.ids:
            i = self.slot_of[d]
            s_i = (i - (self.n - 1) / 2.0) * self.spacing
            ph = self.omega * tau - self.dir * self.k * s_i
            sn, cs = math.sin(ph), math.cos(ph)
            pos, vel, acc = slots[i].copy(), np.zeros(3), np.zeros(3)
            pos[self.wave] += self.A * e * sn
            vel[self.wave] = self.A * (de * sn + e * self.omega * cs)
            acc[self.wave] = self.A * (dde * sn + 2 * de * self.omega * cs - e * self.omega ** 2 * sn)
            out[d] = Setpoint(d, tau, pos, vel, acc, 0.0)
        return out
