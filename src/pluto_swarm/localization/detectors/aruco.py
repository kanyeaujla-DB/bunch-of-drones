"""ArUco marker detector (OpenCV >= 4.7 API: cv2.aruco.ArucoDetector)."""

from __future__ import annotations

from typing import Dict, List

import numpy as np

from ..cameras.base import Frame
from .base import Detection, MarkerDetector


class ArucoDetector(MarkerDetector):
    def __init__(self, cfg: Dict) -> None:
        import cv2                              # localization-only dependency
        self._cv2 = cv2
        mk = cfg.get("marker", {})
        det = cfg.get("detector", {})
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, mk.get("dictionary", "DICT_4X4_50")))
        params = cv2.aruco.DetectorParameters()
        if det.get("corner_refinement", "subpix") == "subpix":
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.minMarkerPerimeterRate = float(det.get("min_marker_perimeter_rate", 0.02))
        self._det = cv2.aruco.ArucoDetector(dictionary, params)

    def detect(self, frame: Frame) -> List[Detection]:
        img = frame.image
        gray = img if img.ndim == 2 else self._cv2.cvtColor(img, self._cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._det.detectMarkers(gray)
        if ids is None:
            return []
        return [Detection(int(i), np.asarray(c, float).reshape(4, 2), frame.t_capture)
                for c, i in zip(corners, ids.ravel())]
