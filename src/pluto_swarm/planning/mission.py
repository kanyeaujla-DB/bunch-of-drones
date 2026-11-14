"""Mission plan: TRANSFER -> FORMATION -> SETTLE, built after take-off.

  TRANSFER   minimum-jerk move from where each drone actually is to its
             assigned formation slot (Hungarian: no crossing paths). All drones
             share ONE duration, so they arrive together.
  FORMATION  the formation's references as functions of mission time tau
             (shared clock; swarm/synchronizer.py may slow tau down).
  SETTLE     hold the final formation positions, then the manager lands.

Missions are described in config/missions/*.yaml (`type` picks the formation).
"""

from __future__ import annotations

from enum import Enum
from typing import Dict, Iterator, List, Optional, Tuple

import numpy as np

from ..core.types import Setpoint
from .assignment import assign_slots
from .formations.circle_wave import CircleWave
from .formations.hold import HoldFormation
from .formations.line_wave import LineWave
from .min_jerk import MinJerk, transfer_duration


class Phase(Enum):
    TRANSFER = "transfer"
    FORMATION = "formation"
    SETTLE = "settle"
    DONE = "done"


def make_formation(mission: Dict, ids: List[int], slot_of: Dict[int, int] = None,
                   hold_slots: Optional[List[np.ndarray]] = None):
    kind = mission.get("type", "hover")
    if kind == "line_wave":
        return LineWave(ids, mission, slot_of)
    if kind == "circle_wave":
        return CircleWave(ids, mission, slot_of)
    if kind in ("hover", "position_hold"):
        return HoldFormation(ids, mission, slot_of, hold_slots)
    raise ValueError(f"unknown mission type {kind!r}")


class MissionPlan:
    def __init__(self, mission: Dict, start: Dict[int, np.ndarray], limits: Dict) -> None:
        """start: where each drone is now (after take-off). limits: v_max, a_max, t_min."""
        self.mission = mission
        ids = sorted(start)
        kind = mission.get("type", "hover")
        if kind in ("hover", "position_hold"):
            h = float(mission.get("height_m", 1.0))
            slots = [np.array([p[0], p[1], h]) for p in (start[i] for i in ids)]
            if "positions" in mission:
                slots = [np.asarray(p, float) for p in mission["positions"]]
            self.formation = make_formation(mission, ids, None, slots)
            slot_of = assign_slots(start, self.formation.slot_positions())
            self.formation.slot_of = slot_of
        else:
            probe = make_formation(mission, ids)               # slot geometry only
            slot_of = assign_slots(start, probe.slot_positions())
            self.formation = make_formation(mission, ids, slot_of)
        self.slot_of = slot_of
        first = self.formation.setpoints(0.0)
        dist = max(float(np.linalg.norm(first[i].pos - start[i])) for i in ids)
        self.t_transfer = transfer_duration(dist, float(limits.get("v_max", 0.4)),
                                            float(limits.get("a_max", 0.8)),
                                            float(limits.get("t_min", 2.0)))
        self.transfers = {i: MinJerk(start[i], first[i].pos, self.t_transfer) for i in ids}
        self.t_settle = float(mission.get("settle_s", 1.5))
        self.ids = ids

    @property
    def formation_duration(self) -> float:
        return self.formation.duration

    def reference(self, phase: Phase, t_phase: float, tau: float) -> Dict[int, Setpoint]:
        if phase == Phase.TRANSFER:
            out = {}
            for i, mj in self.transfers.items():
                p, v, a = mj.at(t_phase)
                out[i] = Setpoint(i, t_phase, p, v, a)
            return out
        if phase == Phase.FORMATION:
            return self.formation.setpoints(tau)
        end = self.formation.setpoints(self.formation.duration)
        return {i: Setpoint(i, t_phase, s.pos.copy()) for i, s in end.items()}

    def sample(self, dt: float = 0.05) -> Iterator[Tuple[Phase, float, Dict[int, Setpoint]]]:
        """Whole plan at nominal clock rate - for pre-flight validation."""
        t = 0.0
        while t <= self.t_transfer:
            yield Phase.TRANSFER, t, self.reference(Phase.TRANSFER, t, 0.0)
            t += dt
        tau = 0.0
        while tau <= self.formation_duration:
            yield Phase.FORMATION, tau, self.reference(Phase.FORMATION, 0.0, tau)
            tau += dt
        yield Phase.SETTLE, 0.0, self.reference(Phase.SETTLE, 0.0, 0.0)
