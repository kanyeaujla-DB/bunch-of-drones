"""Performance metrics from a flight log - the numbers the 10-page report needs.

    python -m pluto_swarm.monitoring.metrics logs/<file>.csv

  tracking error   |actual - reference| per drone during FORMATION (RMS, max)
                   actual = simulator truth if logged, else the filter estimate
  separation       minimum centre-to-centre distance between any pair
  synchronisation  each drone's lag behind its own Z reference (cross-correlation),
                   and the spread of those lags across the swarm (ms)
  drift            final drift-state estimate per drone (m/s^2)
"""

from __future__ import annotations

import csv
import json
import sys
from collections import defaultdict
from typing import Dict

import numpy as np


def _f(v: str):
    return float(v) if v not in ("", None) else None


def compute(path: str, phase: str = "formation") -> Dict:
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if r.get("mission_phase") == phase and r.get("ref_x")
                and r.get("mode") == "full"]
    if not rows:
        return {"error": f"no rows in phase {phase!r}"}
    use_truth = rows[0].get("true_x") not in ("", None)
    src = "true" if use_truth else "est"
    per = defaultdict(lambda: {"t": [], "act": [], "ref": [], "drift": None})
    ticks = defaultdict(dict)
    for r in rows:
        d = int(r["drone"])
        act = np.array([_f(r[f"{src}_x"]), _f(r[f"{src}_y"]), _f(r[f"{src}_z"])])
        ref = np.array([_f(r["ref_x"]), _f(r["ref_y"]), _f(r["ref_z"])])
        per[d]["t"].append(_f(r["t"]))
        per[d]["act"].append(act)
        per[d]["ref"].append(ref)
        per[d]["drift"] = [_f(r["drift_x"]), _f(r["drift_y"]), _f(r["drift_z"])]
        if r.get("tick"):
            ticks[int(r["tick"])][d] = act
    out = {"source": src, "phase": phase, "drones": {}}
    lags = []
    for d, v in sorted(per.items()):
        t = np.array(v["t"])
        A, R = np.array(v["act"]), np.array(v["ref"])
        e = np.linalg.norm(A - R, axis=1)
        ez = np.abs(A[:, 2] - R[:, 2])
        exy = np.linalg.norm(A[:, :2] - R[:, :2], axis=1)
        lag = _lag(t, A[:, 2], R[:, 2])
        lags.append(lag)
        out["drones"][d] = {"rms_err_m": round(float(np.sqrt(np.mean(e ** 2))), 4),
                            "max_err_m": round(float(e.max()), 4),
                            "max_err_xy_m": round(float(exy.max()), 4),
                            "max_err_z_m": round(float(ez.max()), 4),
                            "z_lag_ms": None if lag is None else round(1000 * lag, 1),
                            "drift_m_s2": v["drift"]}
    sep = float("inf")
    for snap in ticks.values():
        ids = sorted(snap)
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                sep = min(sep, float(np.linalg.norm(snap[ids[a]] - snap[ids[b]])))
    out["min_separation_m"] = None if sep == float("inf") else round(sep, 4)
    good = [l for l in lags if l is not None]
    out["sync_spread_ms"] = round(1000 * (max(good) - min(good)), 1) if len(good) > 1 else None
    out["worst_max_err_m"] = max(v["max_err_m"] for v in out["drones"].values())
    out["worst_rms_err_m"] = max(v["rms_err_m"] for v in out["drones"].values())
    return out


def _lag(t: np.ndarray, a: np.ndarray, r: np.ndarray, max_lag: float = 0.5):
    """Delay of signal a behind r (s), by cross-correlation on a uniform grid."""
    if len(t) < 50 or np.std(r) < 0.02:
        return None
    dt = 0.01
    tg = np.arange(t[0], t[-1], dt)
    ag = np.interp(tg, t, a - np.mean(a))
    rg = np.interp(tg, t, r - np.mean(r))
    n = int(max_lag / dt)
    best, best_k = -np.inf, 0
    for k in range(-n, n + 1):
        if k >= 0:
            c = float(np.dot(ag[k:], rg[:len(rg) - k]))
        else:
            c = float(np.dot(ag[:k], rg[-k:]))
        if c > best:
            best, best_k = c, k
    return best_k * dt


def main() -> None:
    print(json.dumps(compute(sys.argv[1]), indent=2, default=str))


if __name__ == "__main__":
    main()
