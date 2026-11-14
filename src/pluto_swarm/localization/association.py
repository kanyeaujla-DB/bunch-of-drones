"""Marker -> drone association.

Primary key is the ArUco ID (unique per drone, config/swarm.yaml marker_id).
For detections whose ID is unknown or unreliable, `associate()` matches them to
the drones' predicted positions with the Hungarian algorithm and a distance gate,
so a mis-decoded marker cannot teleport a drone's estimate.
"""

from __future__ import annotations

from typing import Dict, Sequence

import numpy as np

from ..core.hungarian import hungarian


def associate(predicted: Dict[int, np.ndarray], measured: Sequence[np.ndarray],
              gate_m: float) -> Dict[int, int]:
    """Match unlabelled measurements to drones. Returns {drone_id: measurement index};
    pairs further apart than gate_m are dropped."""
    ids = list(predicted)
    if not ids or not len(measured):
        return {}
    C = np.array([[np.linalg.norm(np.asarray(m) - predicted[i]) for m in measured] for i in ids])
    out = {}
    for r, c in hungarian(C):
        if C[r, c] <= gate_m:
            out[ids[r]] = c
    return out
