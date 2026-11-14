"""Marker corners (pixels) -> drone position + yaw in the arena frame.

Uses cv2.solvePnP with SOLVEPNP_IPPE_SQUARE (the method made for square
markers). Works for any camera pose - overhead or tilted - because the
extrinsics carry the full camera rotation.

Rolling shutter (C920): the marker's measurement time is the time its centre
row was exposed, not the frame time: t = t_frame + readout * row / height.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from .camera_model import CameraModel, marker_corners_body
from .detectors.base import Detection


@dataclass
class MarkerPose:
    marker_id: int
    pos: np.ndarray          # drone centre, arena frame (m)
    yaw: float               # rad about +Z
    t: float                 # measurement time (rolling-shutter corrected)
    reproj_px: float         # RMS corner re-projection error
    range_m: float           # camera -> marker distance
    size_px: float           # mean marker edge length in pixels


class PoseSolver:
    def __init__(self, cam: CameraModel, loc_cfg: Dict) -> None:
        import cv2
        self._cv2 = cv2
        self.cam = cam
        mk = loc_cfg.get("marker", {})
        self.size = float(mk.get("size_m", 0.08))
        self.h_off = float(mk.get("height_above_centre_m", 0.03))
        self.max_reproj = float(loc_cfg.get("detector", {}).get("max_reprojection_px", 3.0))
        self.obj = marker_corners_body(self.size).astype(np.float64)

    def solve(self, det: Detection) -> Optional[MarkerPose]:
        cv2 = self._cv2
        img = det.corners_px.reshape(4, 1, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(self.obj, img, self.cam.K, self.cam.dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        proj, _ = cv2.projectPoints(self.obj, rvec, tvec, self.cam.K, self.cam.dist)
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - det.corners_px) ** 2, axis=1))))
        if err > self.max_reproj:
            return None
        R_cm, _ = cv2.Rodrigues(rvec)
        R_wm = self.cam.R_wc @ R_cm
        if R_wm[2, 2] < 0.5:                 # marker must face up (tilt < 60 deg); else flipped solution
            return None
        p_marker = self.cam.R_wc @ tvec.ravel() + self.cam.t_wc
        pos = p_marker - R_wm @ np.array([0.0, 0.0, self.h_off])
        yaw = float(np.arctan2(R_wm[1, 0], R_wm[0, 0]))
        row = float(np.mean(det.corners_px[:, 1]))
        edges = np.linalg.norm(np.roll(det.corners_px, -1, axis=0) - det.corners_px, axis=1)
        return MarkerPose(det.marker_id, pos, yaw, det.t_capture + self.cam.row_time(row),
                          err, float(np.linalg.norm(tvec)), float(np.mean(edges)))

    def covariance(self, mp: MarkerPose, base_xy: float, base_z: float) -> np.ndarray:
        """3x3 measurement covariance in the arena frame, from geometry.
        Along the viewing ray the error is ~ range * sigma_px / size_px (scale);
        across it, ~ range * sigma_px / f. Rotated into the arena frame."""
        f = float(self.cam.K[0, 0])
        s_px = 0.4
        along = max(base_z, mp.range_m * s_px / max(mp.size_px, 1.0))
        across = max(base_xy, mp.range_m * s_px / f)
        ray = mp.pos - self.cam.t_wc
        ray /= np.linalg.norm(ray)
        cov = across ** 2 * np.eye(3) + (along ** 2 - across ** 2) * np.outer(ray, ray)
        return cov
