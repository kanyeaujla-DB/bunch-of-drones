"""Intel RealSense (D455f) - COLOUR STREAM ONLY. Depth is deliberately not
started, so the algorithm is identical to the C920 one (config/camera/d455_rgb.yaml).

Uses the device's hardware frame timestamp, converted to our clock once at start.
"""

from __future__ import annotations

import logging
import threading
from typing import Dict, Optional

from ...core.clock import now
from .base import CameraSource, Frame

log = logging.getLogger(__name__)


class RealSenseCamera(CameraSource):
    def __init__(self, cam_file: Dict, serial: Optional[str] = None) -> None:
        self.cfg = cam_file
        self.serial = serial
        self._pipe = None
        self._latest: Optional[Frame] = None
        self._seq = self._read_seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._offset: Optional[float] = None       # our clock - device clock (s)

    def open(self) -> None:
        try:
            import pyrealsense2 as rs
        except ImportError as e:                    # pragma: no cover
            raise RuntimeError("final_demo camera needs `pip install pyrealsense2`") from e
        if self.cfg.get("enable_depth"):
            raise RuntimeError("d455_rgb.yaml must keep enable_depth: false (same algorithm as C920)")
        self._rs = rs
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color, int(self.cfg["width"]), int(self.cfg["height"]),
                          rs.format.bgr8, int(self.cfg.get("fps", 60)))
        self._pipe = rs.pipeline()
        profile = self._pipe.start(cfg)
        sensor = profile.get_device().first_color_sensor()
        for name, val in (self.cfg.get("rs_options") or {}).items():
            opt = getattr(rs.option, name, None)
            if opt is not None and sensor.supports(opt):
                sensor.set_option(opt, float(val))
            else:
                log.warning("RealSense option %s not supported", name)
        self._stop.clear()
        self._thread = threading.Thread(target=self._grab, daemon=True, name="realsense")
        self._thread.start()

    def _grab(self) -> None:
        import numpy as np
        while not self._stop.is_set():
            try:
                frames = self._pipe.wait_for_frames(1000)
            except RuntimeError:
                continue
            c = frames.get_color_frame()
            if not c:
                continue
            t_dev = c.get_timestamp() / 1000.0
            t_now = now()
            if self._offset is None:
                self._offset = t_now - t_dev
            t = t_dev + self._offset
            with self._lock:
                self._seq += 1
                self._latest = Frame(np.asanyarray(c.get_data()).copy(), t, "realsense")

    def read(self) -> Optional[Frame]:
        with self._lock:
            if self._latest is None or self._seq == self._read_seq:
                return None
            self._read_seq = self._seq
            return self._latest

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(1.0)
        if self._pipe is not None:
            self._pipe.stop()
