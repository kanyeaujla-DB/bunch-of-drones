"""CSV flight logger. Every run writes logs/<timestamp>_<profile>_<mission>.csv.

These logs ARE the evidence for the report (architecture doc §3): plots are made
from them by tools/, never by hand.
"""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Dict, Optional

from ..core.clock import now

COLUMNS = ["t", "drone", "phase", "roll_deg", "pitch_deg", "yaw_deg", "alt_m", "vario_m_s",
           "battery_v", "rc_roll", "rc_pitch", "rc_throttle", "rc_yaw", "rc_aux4",
           "true_x", "true_y", "true_z"]


class DataLogger:
    def __init__(self, log_dir: Path, tag: str) -> None:
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"{stamp}_{tag}.csv"
        self._f = open(self.path, "w", newline="")
        self._w = csv.DictWriter(self._f, fieldnames=COLUMNS)
        self._w.writeheader()
        self.t0 = now()
        self.rows = 0

    def log(self, drone_id: int, phase: str, link, truth: Optional[Dict] = None) -> None:
        import math
        tel, rc = link.telemetry, link.rc
        deg = lambda r: "" if r is None else f"{math.degrees(r):.2f}"
        num = lambda v, f=".3f": "" if v is None else format(v, f)
        row = {"t": f"{now() - self.t0:.3f}", "drone": drone_id, "phase": phase,
               "roll_deg": deg(tel.roll_rad), "pitch_deg": deg(tel.pitch_rad),
               "yaw_deg": deg(tel.yaw_rad), "alt_m": num(tel.alt_m),
               "vario_m_s": num(tel.vario_m_s), "battery_v": num(tel.battery_v, ".2f"),
               "rc_roll": rc.roll, "rc_pitch": rc.pitch, "rc_throttle": rc.throttle,
               "rc_yaw": rc.yaw, "rc_aux4": rc.aux4,
               "true_x": "", "true_y": "", "true_z": ""}
        if truth is not None:
            row["true_x"], row["true_y"], row["true_z"] = (f"{v:.3f}" for v in truth["pos"])
        self._w.writerow(row)
        self.rows += 1

    def close(self) -> None:
        self._f.close()
