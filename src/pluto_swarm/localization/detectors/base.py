"""MarkerDetector - swappable detector (ArUco default; WhyCon/AprilTag optional)."""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import List

import numpy as np

from ..cameras.base import Frame


@dataclass
class Detection:
    marker_id: int               # -> drone id via config/swarm.yaml marker_id
    corners_px: np.ndarray       # 4 x 2, sub-pixel refined
    t_capture: float             # copied from the Frame


class MarkerDetector(abc.ABC):
    @abc.abstractmethod
    def detect(self, frame: Frame) -> List[Detection]: ...
