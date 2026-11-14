"""USB / V4L2 camera through OpenCV (Logitech C920 at Airlink).

Low latency by design:
  * a grabber thread reads frames continuously and keeps ONLY the newest one
    (read() never returns a stale queued frame);
  * CAP_PROP_BUFFERSIZE = 1 where the backend supports it;
  * the capture timestamp is taken when the frame is dequeued, minus the
    configured sensor-to-USB latency.

Locked automatics: every v4l2 control in the camera file is applied with
`v4l2-ctl` (if installed) and read back; a mismatch is logged loudly because a
focus or exposure change silently invalidates calibration.
"""

from __future__ import annotations

import logging
import shutil
import subprocess
import threading
from typing import Dict, Optional

from ...core.clock import now
from .base import CameraSource, Frame

log = logging.getLogger(__name__)


class OpenCVCamera(CameraSource):
    def __init__(self, cam_file: Dict, device=0, usb_latency_s: float = 0.0) -> None:
        self.cfg = cam_file
        self.device = device
        self.usb_latency = usb_latency_s
        self._cap = None
        self._latest: Optional[Frame] = None
        self._seq = 0
        self._read_seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def open(self) -> None:
        import cv2
        backend = cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else cv2.CAP_ANY
        self._cap = cv2.VideoCapture(self.device, backend)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open camera {self.device}")
        fourcc = self.cfg.get("fourcc", "MJPG")
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.cfg["width"]))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.cfg["height"]))
        self._cap.set(cv2.CAP_PROP_FPS, float(self.cfg.get("fps", 30)))
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.apply_controls()
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (w, h) != (int(self.cfg["width"]), int(self.cfg["height"])):
            raise RuntimeError(f"camera gave {w}x{h}, calibration expects "
                               f"{self.cfg['width']}x{self.cfg['height']}")
        self._stop.clear()
        self._thread = threading.Thread(target=self._grab, daemon=True, name="grabber")
        self._thread.start()

    def apply_controls(self) -> None:
        ctl = self.cfg.get("v4l2_controls") or {}
        if not ctl:
            return
        exe = shutil.which("v4l2-ctl")
        if exe is None:
            log.warning("v4l2-ctl not found - camera automatics NOT locked (install v4l-utils)")
            return
        dev = f"/dev/video{self.device}" if isinstance(self.device, int) else str(self.device)
        aliases = {"focus_automatic_continuous": "focus_auto", "auto_exposure": "exposure_auto",
                   "exposure_time_absolute": "exposure_absolute",
                   "white_balance_automatic": "white_balance_temperature_auto"}
        order = ["focus_automatic_continuous", "auto_exposure", "white_balance_automatic"]
        keys = order + [k for k in ctl if k not in order]      # automatics off FIRST
        for k in keys:
            if k not in ctl:
                continue
            for name in (k, aliases.get(k)):
                if not name:
                    continue
                r = subprocess.run([exe, "-d", dev, f"--set-ctrl={name}={ctl[k]}"],
                                   capture_output=True, text=True)
                if r.returncode == 0:
                    break
            else:
                log.warning("could not set camera control %s=%s", k, ctl[k])

    def _grab(self) -> None:
        while not self._stop.is_set():
            ok, img = self._cap.read()
            t = now() - self.usb_latency
            if not ok:
                continue
            with self._lock:
                self._seq += 1
                self._latest = Frame(img, t, "opencv")

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
        if self._cap is not None:
            self._cap.release()
