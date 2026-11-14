"""Camera geometry, ArUco detection and pose - needs OpenCV (the `vision` extra)."""

import numpy as np
import pytest
import yaml

cv2 = pytest.importorskip("cv2")

from pluto_swarm.core.config import load_config, repo_root  # noqa: E402
from pluto_swarm.localization.association import associate  # noqa: E402
from pluto_swarm.localization.camera_model import load_camera_model, look_at  # noqa: E402
from pluto_swarm.localization.cameras.synthetic_cam import SyntheticCamera  # noqa: E402
from pluto_swarm.localization.detectors.aruco import ArucoDetector  # noqa: E402
from pluto_swarm.localization.pose import PoseSolver  # noqa: E402

ROOT = repo_root()
LOC = yaml.safe_load((ROOT / "config/localization.yaml").read_text())
CFG = load_config("sim", "hover_test")
CAM = load_camera_model(ROOT, CFG.profile["camera"])
TRUTH = {1: {"pos": np.array([1.2, 1.3, 1.0]), "vel": np.zeros(3), "yaw": 0.4},
         2: {"pos": np.array([2.0, 1.7, 1.3]), "vel": np.zeros(3), "yaw": -1.0}}


def test_look_at_points_the_optical_axis_at_the_target():
    R = look_at([1.5, 1.5, 3.5], [1.5, 1.5, 0.0])
    assert np.allclose(R[:, 2], [0, 0, -1])
    assert np.allclose(R.T @ R, np.eye(3))


def test_pose_from_synthetic_corners_is_accurate():
    cam = SyntheticCamera(CAM, lambda: TRUTH, {1: 1, 2: 2}, LOC, {"pixel_noise_px": 0.3, "dropout_prob": 0},
                          rng=np.random.default_rng(0))
    solver = PoseSolver(CAM, LOC)
    err = []
    for _ in range(100):
        for d in cam.capture(0.0):
            mp = solver.solve(d)
            err.append(np.r_[mp.pos - TRUTH[d.marker_id]["pos"], mp.yaw - TRUTH[d.marker_id]["yaw"]])
    err = np.array(err)
    assert np.all(np.abs(err.mean(0)) < [0.005, 0.005, 0.01, 0.01])
    assert np.all(err.std(0) < [0.01, 0.01, 0.03, 0.02])


def test_aruco_detector_finds_rendered_markers():
    cam = SyntheticCamera(CAM, lambda: TRUTH, {1: 1, 2: 2}, LOC)
    dets = ArucoDetector(LOC).detect(cam.render(0.0))
    assert sorted(d.marker_id for d in dets) == [1, 2]
    solver = PoseSolver(CAM, LOC)
    for d in dets:
        assert np.linalg.norm(solver.solve(d).pos - TRUTH[d.marker_id]["pos"]) < 0.03


def test_association_rejects_far_matches():
    pred = {1: np.array([1.0, 1.0, 1.0]), 2: np.array([2.0, 1.0, 1.0])}
    meas = [np.array([2.02, 1.0, 1.0]), np.array([1.0, 2.0, 1.0])]
    assert associate(pred, meas, gate_m=0.15) == {2: 0}
