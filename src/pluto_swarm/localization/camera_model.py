"""CameraModel - intrinsics + extrinsics of one camera, from config/camera/*.yaml.

Frames
  * camera (OpenCV): x right, y down, z forward (out of the lens);
  * arena: origin at a floor corner, Z up (core/types.py).
R_wc, t_wc map camera -> arena:  p_arena = R_wc @ p_cam + t_wc.

If the intrinsics are not calibrated yet, a pinhole model is built from the
nominal focal length in the camera file (good enough for simulation; never for
flight).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence

import numpy as np
import yaml


@dataclass
class CameraModel:
    K: np.ndarray                 # 3x3
    dist: np.ndarray              # (5,)
    width: int
    height: int
    fps: float
    readout_s: float              # rolling-shutter top->bottom time (0 = global)
    R_wc: np.ndarray              # 3x3 camera -> arena
    t_wc: np.ndarray              # (3,) camera position in arena
    calibrated: bool = False

    # ---- geometry -----------------------------------------------------------
    def world_to_cam(self, p_world: np.ndarray) -> np.ndarray:
        return (np.asarray(p_world) - self.t_wc) @ self.R_wc     # = R_wc^T (p - t)

    def project(self, p_world: np.ndarray) -> np.ndarray:
        """N x 3 arena points -> N x 2 pixels (no distortion; for simulation)."""
        pc = self.world_to_cam(np.atleast_2d(p_world))
        uv = (self.K @ (pc / pc[:, 2:3]).T).T
        return uv[:, :2]

    def in_view(self, px: np.ndarray, margin: float = 2.0) -> bool:
        px = np.atleast_2d(px)
        return bool(np.all((px[:, 0] > margin) & (px[:, 0] < self.width - margin) &
                           (px[:, 1] > margin) & (px[:, 1] < self.height - margin)))

    def row_time(self, v_px: float) -> float:
        """Rolling shutter: seconds after frame start at which pixel row v was exposed."""
        return self.readout_s * float(np.clip(v_px / self.height, 0.0, 1.0))


def look_at(position: Sequence[float], target: Sequence[float],
            up_hint: Sequence[float] = (0.0, 1.0, 0.0)) -> np.ndarray:
    """R_wc for a camera at `position` looking at `target`. The image 'up'
    (-y_cam) is aligned as well as possible with up_hint (arena frame)."""
    z = np.asarray(target, float) - np.asarray(position, float)
    z /= np.linalg.norm(z)
    up = np.asarray(up_hint, float)
    x = np.cross(-up, z)                  # y_cam points DOWN in the image
    if np.linalg.norm(x) < 1e-6:          # looking along up_hint: pick any x
        x = np.cross([1.0, 0.0, 0.0], z) if abs(z[0]) < 0.9 else np.cross([0.0, 1.0, 0.0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _intrinsics(cam: Dict) -> tuple:
    w, h = int(cam["width"]), int(cam["height"])
    intr = cam.get("intrinsics") or {}
    if intr.get("calibrated") and intr.get("camera_matrix") is not None:
        K = np.asarray(intr["camera_matrix"], float).reshape(3, 3)
        dist = np.asarray(intr.get("dist_coeffs") or [0, 0, 0, 0, 0], float).ravel()
        return K, dist, True
    nom = cam.get("nominal", {})
    fx = float(nom.get("fx_px", w))
    fy = float(nom.get("fy_px", fx))
    K = np.array([[fx, 0, w / 2.0], [0, fy, h / 2.0], [0, 0, 1.0]])
    return K, np.zeros(5), False


def load_camera_model(root: Path, camera_cfg: Dict) -> CameraModel:
    """camera_cfg = profile['camera'] (needs `model`; pose from `extrinsics` file
    or an inline `pose: {position, look_at, up}` used by simulation profiles)."""
    cam = yaml.safe_load((root / camera_cfg["model"]).read_text())
    K, dist, calibrated = _intrinsics(cam)
    if "pose" in camera_cfg:
        pose = camera_cfg["pose"]
        t = np.asarray(pose["position"], float)
        R = look_at(t, pose["look_at"], pose.get("up", (0.0, 1.0, 0.0)))
    else:
        ext = yaml.safe_load((root / camera_cfg["extrinsics"]).read_text())
        if not ext.get("calibrated"):
            raise RuntimeError(f"{camera_cfg['extrinsics']} is not calibrated - run "
                               "`python -m pluto_swarm.apps.calibrate extrinsics` on site first")
        R = np.asarray(ext["R_arena_cam"], float).reshape(3, 3)
        t = np.asarray(ext["t_arena_cam_m"], float)
    return CameraModel(K=K, dist=dist, width=int(cam["width"]), height=int(cam["height"]),
                       fps=float(cam.get("fps", 30)), readout_s=float(cam.get("readout_s", 0.0)),
                       R_wc=R, t_wc=t, calibrated=calibrated)


def marker_corners_body(size_m: float) -> np.ndarray:
    """Marker corners in the marker frame, OpenCV ArUco order
    (top-left, top-right, bottom-right, bottom-left), z = 0."""
    s = size_m / 2.0
    return np.array([[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]])


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def marker_corners_world(pos: np.ndarray, yaw: float, size_m: float,
                         height_above_centre_m: float, tilt: Optional[np.ndarray] = None) -> np.ndarray:
    """Arena-frame corners of a drone's top marker (marker frame = body frame)."""
    R = rot_z(yaw) if tilt is None else tilt
    centre = np.asarray(pos, float) + np.array([0.0, 0.0, height_above_centre_m])
    return centre + marker_corners_body(size_m) @ R.T
