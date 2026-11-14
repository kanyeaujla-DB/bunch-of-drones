"""Runtime safety filter - control barrier functions (Wang, Ames & Egerstedt).

Minimally changes the commanded accelerations so that every pair of drones
keeps its separation, even when both brake as hard as allowed:

    min_u  sum_i |u_i - u_hat_i|^2   s.t.  h_dot_ij + gamma h_ij^3 >= 0   for every close pair

    h_ij = sqrt(4 a_max (r_ij - D)) + r_dot_ij,   r_ij = |S (p_i - p_j)|

S = diag(1, 1, r_xy / r_z) stretches height differences, so the separation
region is the downwash ellipsoid (Preiss et al.): a drone directly above
another needs D * r_z / r_xy of vertical clearance.

Solved with Hildreth's dual coordinate ascent (exact for this QP, no
dependencies, < 1 ms for 10 drones). In normal flight no constraint is active
and the commands pass through unchanged; every intervention is counted.
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np


class SafetyFilter:
    def __init__(self, safety: Dict) -> None:
        c = safety.get("cbf", {})
        self.enabled = bool(c.get("enabled", True))
        self.D = float(c.get("separation_m", safety.get("min_separation_m", 0.514)) * c.get("filter_margin", 0.85))
        self.r_ratio = float(c.get("downwash_xy_over_z", 0.4))        # r_xy / r_z
        self.a_max = float(c.get("a_max_m_s2", 1.5))
        self.gamma = float(c.get("gamma", 5.0))
        self.radius = float(c.get("neighbour_radius_m", 1.5))
        self.iters = int(c.get("iterations", 50))
        self.S = np.diag([1.0, 1.0, self.r_ratio])
        self.interventions = 0
        self.last_active_pairs: List[Tuple[int, int]] = []

    def filter(self, pos: Dict[int, np.ndarray], vel: Dict[int, np.ndarray],
               u_hat: Dict[int, np.ndarray]) -> Dict[int, np.ndarray]:
        """pos/vel/u_hat: drone id -> 3-vectors (u = desired acceleration, arena)."""
        ids = list(u_hat)
        if not self.enabled or len(ids) < 2:
            return {i: u_hat[i].copy() for i in ids}
        idx = {d: k for k, d in enumerate(ids)}
        n = len(ids)
        rows, rhs, pairs = [], [], []
        for a in range(n):
            for b in range(a + 1, n):
                i, j = ids[a], ids[b]
                dp = self.S @ (pos[i] - pos[j])
                r = float(np.linalg.norm(dp))
                if r > self.radius or r < 1e-6:
                    continue
                dv = self.S @ (vel[i] - vel[j])
                n_hat = dp / r
                rdot = float(n_hat @ dv)
                gap = r - self.D
                root = math.sqrt(4 * self.a_max * max(gap, 1e-4))
                h = root + rdot if gap > 0 else -1.0 + rdot      # inside: push apart hard
                # h_dot = (2 a rdot)/root + (|dv|^2 - rdot^2)/r + n^T S (u_i - u_j)
                drift = (2 * self.a_max * rdot / root if gap > 0 else 0.0) + (float(dv @ dv) - rdot ** 2) / r
                g = self.S @ n_hat                               # gradient wrt u_i (u_j: -g)
                row = np.zeros(3 * n)
                row[3 * idx[i]:3 * idx[i] + 3] = -g             # -g^T (u_i - u_j) <= drift + gamma h^3
                row[3 * idx[j]:3 * idx[j] + 3] = g
                rows.append(row)
                rhs.append(drift + self.gamma * h ** 3)
                pairs.append((i, j))
        u0 = np.concatenate([u_hat[i] for i in ids])
        if not rows:
            self.last_active_pairs = []
            return {i: u_hat[i].copy() for i in ids}
        G, hv = np.array(rows), np.array(rhs)
        if np.all(G @ u0 <= hv + 1e-9):
            self.last_active_pairs = []
            return {i: u_hat[i].copy() for i in ids}
        # Hildreth: min |u - u0|^2 s.t. G u <= h  ->  u = u0 - G^T lam, lam >= 0
        lam = np.zeros(len(hv))
        GG = np.einsum("ij,ij->i", G, G)
        for _ in range(self.iters):
            for k in range(len(hv)):
                u = u0 - G.T @ lam
                step = (G[k] @ u - hv[k]) / GG[k]
                lam[k] = max(0.0, lam[k] + step)
        u = u0 - G.T @ lam
        self.interventions += 1
        self.last_active_pairs = [p for p, l in zip(pairs, lam) if l > 1e-9]
        return {d: u[3 * k:3 * k + 3] for k, d in enumerate(ids)}
