"""One shared mission clock for all drones (architecture doc §6, Synchronization).

All references are functions of mission time tau, so drones stay in phase by
construction. P3 adds formation feedback: tau slows down when any drone lags.
"""

from __future__ import annotations

import time
from typing import Optional


def now() -> float:
    """Ground-station time in seconds. Use this everywhere, never time.time()."""
    return time.monotonic()


class MissionClock:
    def __init__(self) -> None:
        self._t_start: Optional[float] = None
        self._tau = 0.0
        self._t_last: Optional[float] = None
        self.rate = 1.0          # d(tau)/dt; P3 formation feedback lowers this

    def start(self, t: Optional[float] = None) -> None:
        t = now() if t is None else t
        self._t_start = self._t_last = t
        self._tau = 0.0

    @property
    def started(self) -> bool:
        return self._t_start is not None

    def tick(self, t: Optional[float] = None) -> float:
        """Advance and return mission time tau."""
        if self._t_last is None:
            raise RuntimeError("MissionClock.start() first")
        t = now() if t is None else t
        self._tau += self.rate * (t - self._t_last)
        self._t_last = t
        return self._tau

    @property
    def tau(self) -> float:
        return self._tau
