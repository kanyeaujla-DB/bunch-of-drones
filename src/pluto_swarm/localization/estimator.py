"""Per-drone Kalman filter with drift states and delayed-measurement handling.

State (arena frame), 10 elements:
    x = [p(3), v(3), d(3), b]
      p  position (m)          v  velocity (m/s)
      d  unknown acceleration bias (m/s^2) - trim error, battery sag, draughts.
         Estimating it IS the drift correction: control cancels it directly.
      b  baro bias (m)  - baro altitude = p_z + b (+ noise)

Model (what the firmware does with our sticks, see control/actuator.py):
    p' = v
    v_xy' = a_xy(u) - c v_xy + d_xy        a_xy from the modelled tilt, c = linear drag
    v_z'  = (vz_cmd(u) - v_z) / tau_z + d_z alt-hold tracks the commanded climb rate
    d' = w_d,  b' = w_b                    random walks

Delayed measurements: the camera sees the past (capture time < now). Every
prediction step is kept for `history_s`; a measurement is inserted at its own
time and the filter re-propagates to now with the inputs that were really
applied. The controller therefore always acts on the PRESENT state.
"""

from __future__ import annotations

import collections
import math
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

import numpy as np

N = 10
IP, IV, ID, IB = slice(0, 3), slice(3, 6), slice(6, 9), 9


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Input:
    a_xy: np.ndarray        # modelled horizontal acceleration (world), m/s^2
    vz_cmd: float           # effective climb-rate command, m/s


ZERO_INPUT = Input(np.zeros(2), 0.0)


class DroneEstimator:
    def __init__(self, cfg: Dict, tau_z: float, drag: float = 0.0) -> None:
        f = cfg.get("filter", cfg)
        self.tau_z = float(tau_z)
        self.drag = float(drag)
        self.q_a = float(f.get("accel_noise_m_s2", 0.6)) ** 2
        self.q_d = float(f.get("bias_walk_m_s2_per_sqrt_s", 0.05)) ** 2
        self.q_b = float(f.get("baro_bias_walk_m_per_sqrt_s", 0.02)) ** 2
        self.r_baro = float(f.get("baro_sigma_m", 0.04)) ** 2
        self.nis_gate = float(f.get("nis_gate", 16.3))
        self.history_s = float(f.get("history_s", 1.0))
        self.max_dt = 1.0 / float(f.get("rate_hz", 50))
        self.yaw_gain = float(f.get("yaw_gain", 0.3))

        self.x = np.zeros(N)
        self.P = np.diag([1.0] * 3 + [0.25] * 3 + [0.3] * 3 + [0.5])
        self.t: Optional[float] = None
        self.u = ZERO_INPUT
        self.initialised = False
        self.yaw = 0.0
        self.yaw_ok = False
        # checkpoints: (t, x, P, u applied from t onward) - state includes all measurements <= t
        self._hist: Deque[Tuple[float, np.ndarray, np.ndarray, Input]] = collections.deque()
        # accepted measurements inside the history window, time-ordered: (t, z, H, R)
        self._meas: Deque[tuple] = collections.deque()
        self.resync_after = int(f.get("resync_after_rejects", 5))
        self.resyncs = 0
        # bookkeeping for health
        self.t_last_cam: Optional[float] = None
        self.rejected_in_row = 0
        self.cam_updates = 0
        self.cam_rejects = 0

    # ------------------------------------------------------------------ model
    def _step(self, x: np.ndarray, P: np.ndarray, u: Input, dt: float):
        F = np.eye(N)
        F[IP, IV] = dt * np.eye(3)
        F[3, 6] = F[4, 7] = F[5, 8] = dt
        F[5, 5] = 1.0 - dt / self.tau_z
        F[3, 3] = F[4, 4] = 1.0 - self.drag * dt
        bu = np.zeros(N)
        bu[3:5] = u.a_xy * dt
        bu[5] = u.vz_cmd * dt / self.tau_z
        bu[0:2] = 0.5 * u.a_xy * dt * dt
        x = F @ x + bu
        Q = np.zeros((N, N))
        Q[IV, IV] = self.q_a * dt * np.eye(3)
        Q[IP, IP] = self.q_a * dt ** 3 / 3.0 * np.eye(3)
        Q[IP, IV] = Q[IV, IP] = self.q_a * dt ** 2 / 2.0 * np.eye(3)
        Q[ID, ID] = self.q_d * dt * np.eye(3)
        Q[IB, IB] = self.q_b * dt
        P = F @ P @ F.T + Q
        return x, P

    def _propagate(self, x, P, u: Input, t0: float, t1: float):
        t = t0
        while t1 - t > 1e-9:
            dt = min(self.max_dt, t1 - t)
            x, P = self._step(x, P, u, dt)
            t += dt
        return x, P

    # ------------------------------------------------------------------ API
    def predict_to(self, t: float, u: Optional[Input] = None) -> None:
        """Advance to time t with the input applied since the last call, then
        remember `u` as the input applied from t onward (one checkpoint per tick)."""
        if not self.initialised:
            self.t = t
            if u is not None:
                self.u = u
            return
        if t > self.t:
            self.x, self.P = self._propagate(self.x, self.P, self.u, self.t, t)
            self.t = t
        if u is not None:
            self.u = u
        self._checkpoint()
        horizon = self.t - self.history_s
        while self._hist and self._hist[0][0] < horizon:
            self._hist.popleft()
        while self._meas and self._meas[0][0] < horizon:
            self._meas.popleft()

    def _checkpoint(self) -> None:
        """State at self.t INCLUDING every measurement with time <= self.t."""
        if self._hist and self._hist[-1][0] == self.t:
            self._hist[-1] = (self.t, self.x.copy(), self.P.copy(), self.u)
        else:
            self._hist.append((self.t, self.x.copy(), self.P.copy(), self.u))

    def _update(self, x, P, z, H, R, gate: bool) -> Tuple[np.ndarray, np.ndarray, bool]:
        y = z - H @ x
        S = H @ P @ H.T + R
        Si = np.linalg.inv(S)
        nis = float(y @ Si @ y)
        if gate and nis > self.nis_gate:
            return x, P, False
        K = P @ H.T @ Si
        x = x + K @ y
        I_KH = np.eye(N) - K @ H
        P = I_KH @ P @ I_KH.T + K @ R @ K.T       # Joseph form: stays symmetric PSD
        return x, P, True

    def _insert(self, t_m: float, z: np.ndarray, H: np.ndarray, R: np.ndarray, gate: bool) -> bool:
        """Apply a measurement taken at t_m. If it is in the past, rewind to the last
        checkpoint before t_m and REPLAY every later measurement and input in time
        order, so no information is lost (out-of-sequence measurement handling)."""
        meas = (t_m, z, H, R)
        if t_m >= self.t:
            if t_m > self.t:
                self.x, self.P = self._propagate(self.x, self.P, self.u, self.t, t_m)
                self.t = t_m
            x, P, ok = self._update(self.x, self.P, z, H, R, gate)
            if ok:
                self.x, self.P = x, P
                self._meas.append(meas)
                self._checkpoint()
            return ok
        hist = list(self._hist)
        k = None
        for i in range(len(hist) - 1, -1, -1):
            if hist[i][0] < t_m:
                k = i
                break
        if k is None:
            return False                                   # older than the history window
        t_cur, x, P, u = hist[k]
        # events after the checkpoint: later checkpoints (input changes) and measurements
        events = [(m[0], 0, m) for m in self._meas if m[0] > t_cur]
        events.append((t_m, 0, None))                      # None marks the NEW measurement
        events += [(h[0], 1, h) for h in hist[k + 1:]]     # 1: checkpoints sort after meas
        events.sort(key=lambda e: (e[0], e[1]))
        new_hist = hist[:k + 1]
        for te, kind, item in events:
            if te > t_cur:
                x, P = self._propagate(x, P, u, t_cur, te)
                t_cur = te
            if kind == 0:
                zz, HH, RR = (z, H, R) if item is None else item[1:]
                x, P, ok = self._update(x, P, zz, HH, RR, gate and item is None)
                if item is None and not ok:
                    return False                           # rejected: leave everything as it was
            else:
                u = item[3]
                new_hist.append((te, x.copy(), P.copy(), u))
        if self.t > t_cur:
            x, P = self._propagate(x, P, u, t_cur, self.t)
        self.x, self.P = x, P
        self._hist = collections.deque(new_hist)
        ms = list(self._meas) + [meas]
        ms.sort(key=lambda m: m[0])
        self._meas = collections.deque(ms)
        return True

    def update_camera(self, pos: np.ndarray, R: np.ndarray, t_meas: float,
                      yaw: Optional[float] = None, baro_now: Optional[float] = None) -> bool:
        """Position fix from the camera (arena frame). Returns False if rejected."""
        if not self.initialised:
            self.x[:] = 0.0
            self.x[IP] = pos
            self.P = np.diag([0.0] * 3 + [0.25] * 3 + [0.3] * 3 + [0.5])
            self.P[IP, IP] = np.asarray(R, float)
            if baro_now is not None:
                self.x[IB] = baro_now - pos[2]
            self.t = max(self.t or t_meas, t_meas)
            self.initialised = True
            self.t_last_cam = t_meas
            if yaw is not None:
                self.yaw, self.yaw_ok = yaw, True
            self._hist.clear()
            self._meas.clear()
            self._checkpoint()
            return True
        H = np.zeros((3, N))
        H[:, IP] = np.eye(3)
        ok = self._insert(t_meas, np.asarray(pos, float), H, np.asarray(R, float), True)
        if ok:
            self.cam_updates += 1
            self.rejected_in_row = 0
            self.t_last_cam = max(self.t_last_cam or t_meas, t_meas)
            if yaw is not None:
                self.yaw = wrap(self.yaw + self.yaw_gain * wrap(yaw - self.yaw)) if self.yaw_ok else yaw
                self.yaw_ok = True
        else:
            self.cam_rejects += 1
            self.rejected_in_row += 1
            if self.rejected_in_row >= self.resync_after:
                # persistent disagreement: trust the camera again rather than lock out forever
                self.P[IP, IP] += np.eye(3) * 0.05 ** 2
                self.P[IV, IV] += np.eye(3) * 0.2 ** 2
                ok = self._insert(t_meas, np.asarray(pos, float), H, np.asarray(R, float), False)
                if ok:
                    self.resyncs += 1
                    self.rejected_in_row = 0
                    self.t_last_cam = max(self.t_last_cam or t_meas, t_meas)
        return ok

    def update_baro(self, alt: float, t_meas: float) -> bool:
        if not self.initialised:
            return False
        H = np.zeros((1, N))
        H[0, 2] = 1.0
        H[0, IB] = 1.0
        R = np.array([[self.r_baro]])
        return self._insert(t_meas, np.array([alt]), H, R, True)

    # ------------------------------------------------------------------ outputs
    @property
    def pos(self) -> np.ndarray:
        return self.x[IP].copy()

    @property
    def vel(self) -> np.ndarray:
        return self.x[IV].copy()

    @property
    def bias(self) -> np.ndarray:
        return self.x[ID].copy()

    def pos_std(self) -> np.ndarray:
        return np.sqrt(np.clip(np.diag(self.P)[IP], 0.0, None))

    def peek_ahead(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Position/velocity `dt` seconds ahead with the current input (no state change).
        Used by control to compensate the command delay (Smith-predictor style)."""
        if not self.initialised or dt <= 0:
            return self.pos, self.vel
        x, _ = self._propagate(self.x.copy(), self.P.copy(), self.u, 0.0, dt)
        return x[IP], x[IV]
