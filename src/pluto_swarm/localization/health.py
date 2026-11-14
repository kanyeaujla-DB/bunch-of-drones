"""Tracking health per drone: OK / DEGRADED / LOST (architecture doc §4 step 6, §7).

  OK        fresh camera fix (age < degraded_after_s) and the filter is consistent
  DEGRADED  no fix for degraded_after_s, or 3+ rejected fixes in a row, or the
            position uncertainty grew past max_pos_std_m
  LOST      no fix for lost_after_s, or never initialised
"""

from __future__ import annotations

from typing import Dict

from ..core.types import Health
from .estimator import DroneEstimator


def tracking_health(est: DroneEstimator, t: float, cfg: Dict) -> Health:
    tr = cfg.get("tracking", {})
    degraded_after = float(tr.get("degraded_after_s", 0.15))
    lost_after = float(tr.get("lost_after_s", 0.40))
    max_std = float(tr.get("max_pos_std_m", 0.08))
    if not est.initialised or est.t_last_cam is None:
        return Health.LOST
    age = t - est.t_last_cam
    if age > lost_after:
        return Health.LOST
    if age > degraded_after or est.rejected_in_row >= 3 or float(max(est.pos_std())) > max_std:
        return Health.DEGRADED
    return Health.OK
