"""Minimum-jerk point-to-point move (rest to rest), used for take-off -> slot
and formation -> landing transfers. Peak speed = 1.875 * distance / T."""

from __future__ import annotations

from typing import Tuple

import numpy as np

from .envelopes import smoothstep

PEAK_SPEED_FACTOR = 1.875


class MinJerk:
    def __init__(self, p0, p1, duration_s: float) -> None:
        self.p0 = np.asarray(p0, float)
        self.p1 = np.asarray(p1, float)
        self.T = max(1e-3, float(duration_s))

    def at(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        s, ds, dds, _ = smoothstep(t / self.T)
        d = self.p1 - self.p0
        return self.p0 + s * d, (ds / self.T) * d, (dds / self.T ** 2) * d


def transfer_duration(distance_m: float, v_max: float, a_max: float, t_min: float) -> float:
    """Shortest min-jerk duration respecting peak speed and acceleration
    (peak accel of the quintic = 5.77 * d / T^2)."""
    if distance_m <= 1e-6:
        return t_min
    return max(t_min, PEAK_SPEED_FACTOR * distance_m / v_max, (5.774 * distance_m / a_max) ** 0.5)
