"""Synchronizer - the shared formation clock with formation feedback.

All references are functions of ONE mission time tau, so the swarm is in phase
by construction. Formation feedback (virtual-structure approach, Ren & Beard
2004) slows tau when any drone lags, so a struggling drone slows the whole
formation instead of falling out of phase:

    d tau / dt = 1 / (1 + k_f * max(0, max_i |e_i| - deadzone))

The rate is low-pass filtered (no jerks in the references) and floored, so the
formation never stops completely.
"""

from __future__ import annotations

from typing import Dict, Optional

from ..core.clock import MissionClock, now


class FormationClock:
    def __init__(self, cfg: Dict) -> None:
        self.k_f = float(cfg.get("k_f", 4.0))
        self.deadzone = float(cfg.get("deadzone_m", 0.05))
        self.rate_min = float(cfg.get("rate_min", 0.5))
        self.tau_rate = float(cfg.get("rate_filter_s", 0.5))
        self.enabled = bool(cfg.get("enabled", True))
        self.clock = MissionClock()
        self._t_last: Optional[float] = None
        self.min_rate_seen = 1.0

    def start(self, t: Optional[float] = None) -> None:
        t = now() if t is None else t
        self.clock.start(t)
        self.clock.rate = 1.0
        self._t_last = t

    def tick(self, t: float, max_error_m: float) -> float:
        """Advance with the current worst tracking error; returns tau."""
        if self.enabled and self._t_last is not None:
            target = 1.0 / (1.0 + self.k_f * max(0.0, max_error_m - self.deadzone))
            target = max(self.rate_min, target)
            a = min(1.0, (t - self._t_last) / self.tau_rate) if self.tau_rate > 0 else 1.0
            self.clock.rate += (target - self.clock.rate) * a
            self.min_rate_seen = min(self.min_rate_seen, self.clock.rate)
        self._t_last = t
        return self.clock.tick(t)

    @property
    def tau(self) -> float:
        return self.clock.tau

    @property
    def rate(self) -> float:
        return self.clock.rate
