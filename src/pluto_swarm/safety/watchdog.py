"""Layer 3 - watchdogs and the failsafe policy (architecture doc §7).

Every tick, each drone gets a verdict:

  CONTINUE  all good
  HOLD      tracking DEGRADED: level attitude, hold height (the firmware's own
            angle mode + alt-hold keep it steady for a short while), alert
  LAND      tracking LOST, link/telemetry lost, hard geofence, low battery,
            or DIVERGENCE (tracking error large for too long - e.g. a wrong
            stick sign on the first Airlink flight)

Mission policy: if ANY drone gets LAND, the mission aborts - the others hold
position under closed-loop control and land too ("autonomous flight must not
continue without reliable position feedback", MoM §7).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

import numpy as np

from ..core.types import Health
from .geofence import Geofence


class Verdict(Enum):
    CONTINUE = 0
    HOLD = 1
    LAND = 2


@dataclass
class DroneCheck:
    verdict: Verdict
    reasons: List[str]


class Watchdog:
    def __init__(self, safety: Dict, arena: Dict) -> None:
        self.fence = Geofence(arena)
        self.tel_timeout = float(safety.get("telemetry_timeout_s", 0.5))
        self.land_v = float(safety.get("land_battery_v", 3.4))
        div = safety.get("divergence", {})
        self.div_err = float(div.get("error_m", 0.40))
        self.div_time = float(div.get("time_s", 1.0))
        self._div_since: Dict[int, Optional[float]] = {}
        self.events: List[str] = []

    def check(self, t: float, did: int, health: Health, link, est_pos: np.ndarray,
              ref_pos: Optional[np.ndarray], airborne: bool) -> DroneCheck:
        reasons, verdict = [], Verdict.CONTINUE
        if not link.connected or link.telemetry_age(t) > self.tel_timeout:
            reasons.append("link/telemetry lost (firmware lands on RC loss)")
            verdict = Verdict.LAND
        bv = link.telemetry.battery_v
        if bv is not None and bv < self.land_v:
            reasons.append(f"battery {bv:.2f} V < {self.land_v} V")
            verdict = Verdict.LAND
        if airborne:
            if health == Health.LOST:
                reasons.append("tracking LOST")
                verdict = Verdict.LAND
            elif health == Health.DEGRADED and verdict == Verdict.CONTINUE:
                reasons.append("tracking DEGRADED - holding")
                verdict = Verdict.HOLD
            if health != Health.LOST and self.fence.breached_hard(est_pos):
                reasons.append("hard geofence breached")
                verdict = Verdict.LAND
            if ref_pos is not None and health == Health.OK:
                err = float(np.linalg.norm(ref_pos - est_pos))
                if err > self.div_err:
                    since = self._div_since.get(did) or t
                    self._div_since[did] = since
                    if t - since > self.div_time:
                        reasons.append(f"divergence: error {err:.2f} m for > {self.div_time} s")
                        verdict = Verdict.LAND
                else:
                    self._div_since[did] = None
        return DroneCheck(verdict, reasons)
