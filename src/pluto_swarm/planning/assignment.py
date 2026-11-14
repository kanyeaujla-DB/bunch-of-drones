"""Drone -> formation-slot assignment.

Minimising the SUM OF SQUARED distances (Hungarian) also guarantees that
straight-line transfers never cross (a classic result for the squared-distance
assignment), which keeps take-off -> formation moves collision-free.
"""

from __future__ import annotations

from typing import Dict, Sequence

import numpy as np

from ..core.hungarian import hungarian


def assign_slots(current: Dict[int, np.ndarray], slots: Sequence[np.ndarray],
                 horizontal_only: bool = True) -> Dict[int, int]:
    ids = list(current)
    if len(slots) < len(ids):
        raise ValueError(f"{len(ids)} drones but only {len(slots)} slots")
    dims = slice(0, 2) if horizontal_only else slice(0, 3)
    C = np.array([[float(np.sum((np.asarray(current[i])[dims] - np.asarray(s)[dims]) ** 2))
                   for s in slots] for i in ids])
    return {ids[r]: c for r, c in hungarian(C)}
