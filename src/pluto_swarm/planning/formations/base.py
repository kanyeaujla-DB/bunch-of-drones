"""Formation - a reference generator: mission time tau -> one Setpoint per drone.

Must work for any N (never assume 4). Line and circle formulas: architecture
doc §6. Every reference is a smooth function of the shared mission clock tau,
so synchronisation is built in.
"""

from __future__ import annotations

import abc
from typing import Dict, List

from ...core.types import Setpoint


class Formation(abc.ABC):
    def __init__(self, drone_ids: List[int], params: Dict) -> None:
        self.ids = drone_ids
        self.params = params

    @abc.abstractmethod
    def setpoints(self, tau: float) -> Dict[int, Setpoint]:
        """pos, vel AND acc (feed-forward keeps the wave in phase)."""
