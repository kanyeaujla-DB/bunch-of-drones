"""Localizer - camera detections + baro telemetry -> one DroneState per drone.

Data flow (architecture doc §4):
    CameraSource --frame--> MarkerDetector --corners--> PoseSolver --pose-->
    association --> DroneEstimator (per drone, delay-compensated) --> DroneState

Threading: real cameras run capture+detection in their own thread (so a slow
frame never delays the 50 Hz control tick); the simulator's synthetic camera
runs as an asyncio task. Both call on_detections(); a lock protects the filters.
Control reads state() once per tick after predict().
"""

from __future__ import annotations

import asyncio
import logging
import threading
from typing import Callable, Dict, List, Optional

import numpy as np

from ..core.clock import now
from ..core.types import DroneState
from .association import associate
from .detectors.base import Detection
from .estimator import DroneEstimator, Input
from .health import tracking_health
from .pose import PoseSolver

log = logging.getLogger(__name__)


class Localizer:
    def __init__(self, drone_markers: Dict[int, int], loc_cfg: Dict, safety_cfg: Dict,
                 tau_z: float, pose_solver: Optional[PoseSolver] = None, drag: float = 0.0) -> None:
        self.marker_to_drone = {m: d for d, m in drone_markers.items()}
        self.cfg = loc_cfg
        self.safety_cfg = safety_cfg
        self.solver = pose_solver
        f = loc_cfg.get("filter", {})
        self.base_xy = float(f.get("cam_sigma_xy_m", 0.01))
        self.base_z = float(f.get("cam_sigma_z_m", 0.03))
        self.baro_delay_k = float(loc_cfg.get("timing", {}).get("baro_delay_from_rtt", 0.5))
        self.est: Dict[int, DroneEstimator] = {d: DroneEstimator(loc_cfg, tau_z, drag) for d in drone_markers}
        self._lock = threading.Lock()
        self._last_baro_t: Dict[int, float] = {d: 0.0 for d in drone_markers}
        self._last_baro: Dict[int, Optional[float]] = {d: None for d in drone_markers}
        self.frames = 0
        self.detections = 0
        self.unknown_ids = 0
        self.pose_failures = 0

    # ------------------------------------------------------------ inputs
    def on_detections(self, dets: List[Detection]) -> None:
        """Any thread. Solves poses and feeds the filters."""
        if self.solver is None:
            return
        self.frames += 1
        known, unknown = [], []
        for d in dets:
            mp = self.solver.solve(d)
            if mp is None:
                self.pose_failures += 1
                continue
            (known if d.marker_id in self.marker_to_drone else unknown).append(mp)
        with self._lock:
            for mp in known:
                self._apply(self.marker_to_drone[mp.marker_id], mp)
            if unknown:
                self.unknown_ids += len(unknown)
                pred = {i: e.pos for i, e in self.est.items() if e.initialised}
                for did, k in associate(pred, [m.pos for m in unknown], gate_m=0.15).items():
                    self._apply(did, unknown[k])

    def _apply(self, did: int, mp) -> None:
        R = self.solver.covariance(mp, self.base_xy, self.base_z)
        if self.est[did].update_camera(mp.pos, R, mp.t, mp.yaw, self._last_baro[did]):
            self.detections += 1

    def on_position(self, did: int, pos: np.ndarray, t_meas: float, yaw: Optional[float] = None,
                    sigma_xy: float = 0.01, sigma_z: float = 0.02) -> None:
        """Direct position fix (truth-based debug source, or an external tracker)."""
        R = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_z ** 2])
        with self._lock:
            self.est[did].update_camera(np.asarray(pos, float), R, t_meas, yaw, self._last_baro[did])

    def on_telemetry(self, did: int, tel, rtt_s: Optional[float]) -> None:
        """Main loop, once per tick: feed a NEW baro sample (if any) to the filter."""
        if tel.alt_m is None or tel.t_altitude <= self._last_baro_t[did]:
            return
        self._last_baro_t[did] = tel.t_altitude
        self._last_baro[did] = tel.alt_m
        t_meas = tel.t_altitude - self.baro_delay_k * (rtt_s or 0.0)
        with self._lock:
            self.est[did].update_baro(tel.alt_m, t_meas)

    # ------------------------------------------------------------ per tick
    def predict(self, t: float, inputs: Dict[int, Input]) -> None:
        with self._lock:
            for did, e in self.est.items():
                e.predict_to(t, inputs.get(did))

    def state(self, did: int, t: Optional[float] = None) -> DroneState:
        t = now() if t is None else t
        with self._lock:
            e = self.est[did]
            h = tracking_health(e, t, self.safety_cfg)
            return DroneState(id=did, t=e.t or t, pos=e.pos, vel=e.vel, yaw=e.yaw,
                              bias=e.bias, health=h, source="fused" if e.initialised else "none")

    def peek_ahead(self, did: int, dt: float):
        with self._lock:
            return self.est[did].peek_ahead(dt)


# ---------------------------------------------------------------- runners
class SyntheticRunner:
    """Simulation: grabs detections from SyntheticCamera at its fps and delivers
    them after the camera latency, on the event loop."""

    def __init__(self, cam, localizer: Localizer, fps: float) -> None:
        self.cam, self.loc, self.period = cam, localizer, 1.0 / fps
        self._task: Optional[asyncio.Task] = None

    def start(self) -> None:
        self._task = asyncio.ensure_future(self._loop())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def _loop(self) -> None:
        loop = asyncio.get_running_loop()
        next_t = now()
        while True:
            t_c = now()
            dets = self.cam.capture(t_c)
            loop.call_later(self.cam.latency, self.loc.on_detections, dets)
            next_t += self.period
            await asyncio.sleep(max(0.0, next_t - now()))


class TruthRunner:
    """Debug only (sim): perfect positions with optional noise - isolates control
    problems from localization problems. Never used on hardware."""

    def __init__(self, truth: Callable[[], Dict[int, dict]], localizer: Localizer, fps: float,
                 noise_m: float = 0.0, latency_s: float = 0.0, seed: int = 0) -> None:
        self.truth, self.loc, self.period = truth, localizer, 1.0 / fps
        self.noise, self.latency = noise_m, latency_s
        self.rng = np.random.default_rng(seed)
        self._task: Optional[asyncio.Task] = None

    def start(self) -> None:
        self._task = asyncio.ensure_future(self._loop())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def _loop(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            t_c = now()
            for did, d in self.truth().items():
                p = np.asarray(d["pos"], float) + self.rng.normal(0, self.noise, 3)
                loop.call_later(self.latency, self.loc.on_position, did, p, t_c, d.get("yaw", 0.0),
                                max(self.noise, 0.003), max(self.noise, 0.003))
            await asyncio.sleep(self.period)


class ThreadedCameraRunner:
    """Real camera: capture -> detect -> localizer, in a background thread.
    Always processes the NEWEST frame (old frames are dropped, never queued)."""

    def __init__(self, camera, detector, localizer: Localizer) -> None:
        self.camera, self.detector, self.loc = camera, detector, localizer
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.processed = 0
        self.last_frame_t = 0.0

    def start(self) -> None:
        self.camera.open()
        self._thread = threading.Thread(target=self._run, daemon=True, name="camera")
        self._thread.start()

    async def stop(self) -> None:
        self._stop.set()
        if self._thread:
            await asyncio.get_running_loop().run_in_executor(None, self._thread.join, 2.0)
        self.camera.close()

    def _run(self) -> None:
        import time
        while not self._stop.is_set():
            frame = self.camera.read()
            if frame is None:
                time.sleep(0.002)
                continue
            self.last_frame_t = frame.t_capture
            try:
                self.loc.on_detections(self.detector.detect(frame))
                self.processed += 1
            except Exception as e:           # never let one bad frame kill tracking
                log.warning("camera frame failed: %s", e)
