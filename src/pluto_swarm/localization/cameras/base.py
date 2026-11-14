"""CameraSource - swappable camera edge (localization team owns this folder).

Implementations to add: opencv_cam.py (USB/V4L2), realsense_cam.py,
ros_image_cam.py (Gazebo camera topic), synthetic_cam.py (projects sim truth
through the real calibration, with noise/blur/dropouts).
"""

from __future__ import annotations

import abc
from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    image: np.ndarray        # H x W x 3, BGR (OpenCV order)
    t_capture: float         # core.clock.now() at CAPTURE time, not processing time
    camera_id: str = "cam0"


class CameraSource(abc.ABC):
    @abc.abstractmethod
    def open(self) -> None: ...

    @abc.abstractmethod
    def read(self) -> Frame | None:
        """Latest frame, or None if no new frame is ready. Must not block > 5 ms."""

    @abc.abstractmethod
    def close(self) -> None: ...
