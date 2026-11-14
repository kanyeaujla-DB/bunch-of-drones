"""Geofence (architecture doc §7): soft fence clips references, hard fence lands."""

from __future__ import annotations

from typing import Dict

import numpy as np


class Geofence:
    def __init__(self, arena: Dict) -> None:
        g = arena.get("geofence", {})
        self.lo = np.array([0.0, 0.0, 0.0])
        self.hi = np.array([float(arena["size_x"]), float(arena["size_y"]), float(arena["ceiling"])])
        self.soft = float(g.get("soft_margin", 0.3))
        self.hard = float(g.get("hard_margin", 0.1))
        self.z_min = float(g.get("z_min_flight", 0.5))
        self.z_max = float(g.get("z_max_flight", 2.0))

    def clip(self, p: np.ndarray) -> np.ndarray:
        """Soft fence: pull a reference back inside (horizontal margin + flight band)."""
        q = np.asarray(p, float).copy()
        q[:2] = np.clip(q[:2], self.lo[:2] + self.soft, self.hi[:2] - self.soft)
        q[2] = float(np.clip(q[2], self.z_min, self.z_max))
        return q

    def inside_soft(self, p: np.ndarray, z_check: bool = True) -> bool:
        p = np.asarray(p, float)
        ok = bool(np.all(p[:2] >= self.lo[:2] + self.soft - 1e-9) and
                  np.all(p[:2] <= self.hi[:2] - self.soft + 1e-9))
        if z_check:
            ok = ok and self.z_min - 1e-9 <= p[2] <= self.z_max + 1e-9
        return ok

    def breached_hard(self, p: np.ndarray) -> bool:
        p = np.asarray(p, float)
        return bool(np.any(p[:2] < self.lo[:2] + self.hard) or np.any(p[:2] > self.hi[:2] - self.hard)
                    or p[2] > self.hi[2] - self.hard)
