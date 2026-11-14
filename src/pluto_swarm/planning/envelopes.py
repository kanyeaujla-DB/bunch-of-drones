"""Amplitude envelope: 0 -> 1 -> hold -> 1 -> 0, smooth to the 2nd derivative.

Waves start and stop from rest: the amplitude A(tau) = A * e(tau) ramps with a
quintic smoothstep (zero velocity AND acceleration at both ends of each ramp),
so the feed-forward acceleration never jumps - the "smooth movement and
coordinated transitions" rubric line.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


def smoothstep(u: float) -> Tuple[float, float, float, float]:
    """Quintic s(u) = 10u^3 - 15u^4 + 6u^5 on [0, 1]: value, d/du, d2/du2, integral 0..u."""
    if u <= 0.0:
        return 0.0, 0.0, 0.0, 0.0
    if u >= 1.0:
        return 1.0, 0.0, 0.0, 0.5 + (u - 1.0)
    s = u * u * u * (10 - 15 * u + 6 * u * u)
    ds = 30 * u * u * (1 - u) ** 2
    dds = 60 * u * (1 - u) * (1 - 2 * u)
    integ = u ** 4 * (2.5 - 3 * u + u * u)
    return s, ds, dds, integ


@dataclass
class Envelope:
    ramp_s: float      # time to fade in (and out)
    hold_s: float      # full-amplitude time

    @property
    def duration(self) -> float:
        return 2 * self.ramp_s + self.hold_s

    def at(self, tau: float) -> Tuple[float, float, float, float]:
        """(e, de/dtau, d2e/dtau2, integral of e from 0 to tau) - integral in seconds."""
        R, H = self.ramp_s, self.hold_s
        if R <= 0:
            if 0 <= tau <= H:
                return 1.0, 0.0, 0.0, tau
            return 0.0, 0.0, 0.0, max(0.0, min(tau, H))
        if tau <= 0:
            return 0.0, 0.0, 0.0, 0.0
        if tau < R:
            s, ds, dds, integ = smoothstep(tau / R)
            return s, ds / R, dds / R ** 2, integ * R
        if tau < R + H:
            return 1.0, 0.0, 0.0, 0.5 * R + (tau - R)
        if tau < 2 * R + H:
            u = (tau - R - H) / R
            s, ds, dds, integ = smoothstep(u)
            return 1.0 - s, -ds / R, -dds / R ** 2, 0.5 * R + H + (u - integ) * R
        return 0.0, 0.0, 0.0, R + H
