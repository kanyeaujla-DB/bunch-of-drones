"""Hungarian (Kuhn-Munkres) assignment - shared by localization (marker ->
drone matching) and planning (drone -> formation slot). Dependency-free."""

from __future__ import annotations

from typing import List, Tuple

import numpy as np


def hungarian(cost: np.ndarray) -> List[Tuple[int, int]]:
    """Minimum-cost assignment for an n x m cost matrix (rectangular allowed).
    Returns (row, col) pairs. O(n^2 m), dependency-free (Kuhn-Munkres with potentials)."""
    cost = np.asarray(cost, float)
    transposed = cost.shape[0] > cost.shape[1]
    if transposed:
        cost = cost.T
    n, m = cost.shape
    INF = float("inf")
    u = np.zeros(n + 1)
    v = np.zeros(m + 1)
    p = np.zeros(m + 1, dtype=int)        # p[j] = row (1-based) assigned to column j
    way = np.zeros(m + 1, dtype=int)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = np.full(m + 1, INF)
        used = np.zeros(m + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta, j1 = INF, 0
            for j in range(1, m + 1):
                if not used[j]:
                    cur = cost[i0 - 1, j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j], way[j] = cur, j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    pairs = [(p[j] - 1, j - 1) for j in range(1, m + 1) if p[j] != 0]
    if transposed:
        pairs = [(c, r) for r, c in pairs]
    return sorted(pairs)
