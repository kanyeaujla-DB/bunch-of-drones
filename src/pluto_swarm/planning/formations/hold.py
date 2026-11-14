"""Static formation: every drone holds a fixed point (hover / position-hold tests)."""

from __future__ import annotations

from typing import Dict, List

import numpy as np

from ...core.types import Setpoint
from .base import Formation


class HoldFormation(Formation):
    def __init__(self, drone_ids: List[int], params: Dict, slot_of: Dict[int, int] = None,
                 slots: List[np.ndarray] = None) -> None:
        super().__init__(drone_ids, params)
        self.slot_of = slot_of or {d: i for i, d in enumerate(drone_ids)}
        self._slots = [np.asarray(s, float) for s in slots]
        self._duration = float(params.get("hover_s", params.get("hold_s", 8.0)))

    @property
    def duration(self) -> float:
        return self._duration

    def slot_positions(self) -> List[np.ndarray]:
        return [s.copy() for s in self._slots]

    def setpoints(self, tau: float) -> Dict[int, Setpoint]:
        return {d: Setpoint(d, tau, self._slots[self.slot_of[d]].copy()) for d in self.ids}
