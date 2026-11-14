"""Layer 1 - pre-flight validation of the PLAN (architecture doc §7).

Samples every drone's whole trajectory (transfer + formation + settle) and
refuses to fly if any of these fail:
  * pairwise separation (downwash ellipsoid) >= min_separation_m;
  * inside the soft geofence and the flight-height band;
  * speed, horizontal acceleration (tilt) and climb rate within limits.
Returns a report: list of problems + the minimum pair distance (for the docs).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

from ..planning.mission import MissionPlan, Phase
from .geofence import Geofence


@dataclass
class PlanReport:
    problems: List[str] = field(default_factory=list)
    min_separation_m: float = float("inf")
    max_speed_m_s: float = 0.0
    max_acc_xy_m_s2: float = 0.0
    max_climb_m_s: float = 0.0
    duration_s: float = 0.0

    @property
    def ok(self) -> bool:
        return not self.problems


def validate_plan(plan: MissionPlan, arena: Dict, safety: Dict, plant: Dict, control: Dict,
                  dt: float = 0.05) -> PlanReport:
    rep = PlanReport()
    fence = Geofence(arena)
    D = float(safety.get("min_separation_m", 0.514))
    D_transfer = float(safety.get("limits", {}).get("transfer_min_separation_m", D))
    ratio = float(safety.get("cbf", {}).get("downwash_xy_over_z", 0.4))
    S = np.array([1.0, 1.0, ratio])
    lim = safety.get("limits", {})
    v_max = float(lim.get("v_max_m_s", 0.6))
    g = float(plant.get("g", 9.81))
    tilt = min(float(control.get("position", {}).get("max_tilt_cmd_deg", 10.0)), float(plant["max_tilt_deg"]))
    a_xy_max = 0.8 * g * np.tan(np.radians(tilt))          # keep 20 % for feedback
    climb_max = 0.8 * float(plant["alt_hold_v_max_cm_s"]) / 100.0
    sink_max = 0.8 * -float(plant["alt_hold_v_min_cm_s"]) / 100.0
    seen = set()

    def flag(key: str, msg: str) -> None:
        if key not in seen:
            seen.add(key)
            rep.problems.append(msg)

    n_samples = 0
    start_gap: Dict = {}
    for phase, t, sps in plan.sample(dt):
        n_samples += 1
        ids = sorted(sps)
        for i in ids:
            s = sps[i]
            if phase != Phase.TRANSFER and not fence.inside_soft(s.pos):
                flag(f"fence{i}", f"drone {i}: reference leaves the soft geofence/height band "
                                  f"({phase.value}, t={t:.1f}s, pos={np.round(s.pos, 2)})")
            if phase == Phase.TRANSFER and fence.breached_hard(s.pos):
                flag(f"hfence{i}", f"drone {i}: transfer path crosses the hard geofence")
            sp = float(np.linalg.norm(s.vel))
            axy = float(np.linalg.norm(s.acc[:2]))
            rep.max_speed_m_s = max(rep.max_speed_m_s, sp)
            rep.max_acc_xy_m_s2 = max(rep.max_acc_xy_m_s2, axy)
            rep.max_climb_m_s = max(rep.max_climb_m_s, abs(float(s.vel[2])))
            if sp > v_max:
                flag(f"v{i}", f"drone {i}: speed {sp:.2f} m/s > {v_max} m/s")
            if axy > a_xy_max:
                flag(f"a{i}", f"drone {i}: horizontal accel {axy:.2f} m/s^2 needs > 80 % of the tilt limit")
            if s.vel[2] > climb_max or -s.vel[2] > sink_max:
                flag(f"vz{i}", f"drone {i}: climb rate {s.vel[2]:.2f} m/s outside alt-hold limits")
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                d = float(np.linalg.norm(S * (sps[ids[a]].pos - sps[ids[b]].pos)))
                key = (ids[a], ids[b])
                if phase == Phase.TRANSFER and key not in start_gap:
                    start_gap[key] = d            # where the drones ARE after take-off
                limit = D
                if phase == Phase.TRANSFER:       # may start closer, must never get closer
                    limit = min(D_transfer, start_gap[key] - 0.01)
                if phase != Phase.TRANSFER:
                    rep.min_separation_m = min(rep.min_separation_m, d)
                if d < limit:
                    flag(f"sep{ids[a]}{ids[b]}", f"drones {ids[a]} & {ids[b]}: separation {d:.3f} m "
                                                 f"< {limit:.3f} m ({phase.value}, t={t:.1f}s)")
    rep.duration_s = plan.t_transfer + plan.formation_duration + plan.t_settle
    return rep
