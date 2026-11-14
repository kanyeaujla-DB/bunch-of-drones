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
           "true_x", "true_y", "true_z",
           # closed-loop columns (empty in open-loop runs)
           "tick", "mode", "mission_phase", "tau", "clock_rate", "health",
           "ref_x", "ref_y", "ref_z", "est_x", "est_y", "est_z", "est_vx", "est_vy", "est_vz",
           "est_yaw_deg", "drift_x", "drift_y", "drift_z", "rtt_ms"]


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

    def log(self, drone_id: int, phase: str, link, truth: Optional[Dict] = None,
            flight: Optional[Dict] = None, tick: Optional[int] = None) -> None:
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
        rtt = getattr(link, "rtt_s", None)
        row["rtt_ms"] = "" if rtt is None else f"{1000 * rtt:.1f}"
        if flight:
            row["tick"] = "" if tick is None else tick
            row["mode"], row["mission_phase"], row["health"] = flight["mode"], flight["phase"], flight["health"]
            row["tau"], row["clock_rate"] = f"{flight['tau']:.3f}", f"{flight['rate']:.3f}"
            if flight["ref"] is not None:
                row["ref_x"], row["ref_y"], row["ref_z"] = (f"{v:.3f}" for v in flight["ref"])
            row["est_x"], row["est_y"], row["est_z"] = (f"{v:.3f}" for v in flight["est"])
            row["est_vx"], row["est_vy"], row["est_vz"] = (f"{v:.3f}" for v in flight["vel"])
            row["drift_x"], row["drift_y"], row["drift_z"] = (f"{v:.3f}" for v in flight["bias"])
            row["est_yaw_deg"] = f"{math.degrees(flight['yaw']):.1f}"
        self._w.writerow(row)
        self.rows += 1

    def close(self) -> None:
        self._f.close()
