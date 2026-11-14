"""Synthetic camera for software-in-the-loop tests.

Projects the simulator's TRUE marker corners through the same CameraModel the
real pipeline uses, then adds what a real camera does to them:
  * pixel noise on every corner;
  * rolling-shutter skew (each corner sampled at its own row time);
  * random dropouts (motion blur / decode failure) and out-of-view loss;
  * capture-to-delivery latency (the detection arrives later than t_capture).

Two outputs:
  * capture(t) -> list[Detection]  (fast path: skips image rendering, feeds PoseSolver)
  * render(t)  -> Frame            (full path: a real image for the ArUco detector)
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional

import numpy as np

from ..camera_model import CameraModel, marker_corners_world, rot_z
from ..detectors.base import Detection
from .base import Frame

TruthFn = Callable[[], Dict[int, dict]]     # drone_id -> {"pos", "vel", "yaw", ...}


class SyntheticCamera:
    def __init__(self, cam: CameraModel, truth: TruthFn, marker_of: Dict[int, int],
                 loc_cfg: Dict, sim_cfg: Optional[Dict] = None,
                 rng: Optional[np.random.Generator] = None) -> None:
        self.cam = cam
        self.truth = truth
        self.marker_of = marker_of               # drone id -> marker id
        mk = loc_cfg.get("marker", {})
        self.size = float(mk.get("size_m", 0.08))
        self.h_off = float(mk.get("height_above_centre_m", 0.03))
        s = sim_cfg or {}
        self.px_noise = float(s.get("pixel_noise_px", 0.3))
        self.dropout = float(s.get("dropout_prob", 0.02))
        self.latency = float(s.get("latency_s", loc_cfg.get("timing", {}).get("camera_latency_s", 0.035)))
        self.rng = rng or np.random.default_rng(0)
        self.blind: set = set()                  # drone ids hidden on purpose (fault injection)

    def _corners(self, d: dict, row_dt: Optional[np.ndarray] = None) -> np.ndarray:
        pos = np.asarray(d["pos"], float)
        vel = np.asarray(d.get("vel", np.zeros(3)), float)
        tilt = rot_z(d.get("yaw", 0.0))
        if "phi" in d:                           # include body tilt if the sim reports it
            phi, th = d["phi"], d["theta"]
            Rx = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
            Ry = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]])
            tilt = tilt @ Ry @ Rx
        cw = marker_corners_world(pos, d.get("yaw", 0.0), self.size, self.h_off, tilt)
        if row_dt is not None:                   # rolling shutter: corner seen later = moved further
            cw = cw + np.outer(row_dt, vel)
        return cw

    def capture(self, t_capture: float) -> List[Detection]:
        out: List[Detection] = []
        for did, d in self.truth().items():
            if did in self.blind or did not in self.marker_of:
                continue
            if self.rng.random() < self.dropout:
                continue
            px = self.cam.project(self._corners(d))
            if self.cam.readout_s > 0:
                row_dt = np.array([self.cam.row_time(v) for v in px[:, 1]])
                px = self.cam.project(self._corners(d, row_dt))
            if not self.cam.in_view(px):
                continue
            px = px + self.rng.normal(0.0, self.px_noise, px.shape)
            out.append(Detection(self.marker_of[did], px, t_capture))
        return out

    def render(self, t_capture: float, background: int = 200) -> Frame:
        """Draw every visible marker into a grey image (for detector tests)."""
        import cv2
        img = np.full((self.cam.height, self.cam.width), background, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        n = 240
        for did, d in self.truth().items():
            if did not in self.marker_of:
                continue
            px = self.cam.project(self._corners(d)).astype(np.float32)
            if not self.cam.in_view(px):
                continue
            tag = cv2.aruco.generateImageMarker(dictionary, self.marker_of[did], n)
            src = np.float32([[0, 0], [n - 1, 0], [n - 1, n - 1], [0, n - 1]])
            H = cv2.getPerspectiveTransform(src, px)
            warped = cv2.warpPerspective(tag, H, (self.cam.width, self.cam.height),
                                         flags=cv2.INTER_LINEAR, borderValue=255)
            mask = cv2.warpPerspective(np.full((n, n), 255, np.uint8), H,
                                       (self.cam.width, self.cam.height)) > 127
            img[mask] = warped[mask]
        return Frame(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), t_capture, "synthetic")
