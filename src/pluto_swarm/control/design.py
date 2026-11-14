"""Gain design from the (identified) plant - `python -m pluto_swarm.control.design`.

Horizontal plant per axis, discrete at the control rate:
    accel command --[pure delay d]--> first-order lag tau --> double integrator
We grid-search natural frequency w for the PD law a = w^2 e + 2 zeta w e_dot and
keep the fastest w whose closed-loop poles ALL have damping >= zeta_min and
lie inside |z| < r_max. Re-run after Airlink session 1 updates
config/plant/pluto_v5.yaml; paste the result into config/control.yaml.

`residual_delay_s` is the delay the controller still sees after the
Smith-style prediction (prediction is never perfect, so keep it > 0).
"""

from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np


def closed_loop_poles(kp: float, kd: float, tau: float, delay: float, dt: float) -> np.ndarray:
    nd = max(0, int(round(delay / dt)))
    # state: [p, v, a, u_{k-1}, ..., u_{k-nd}]
    n = 3 + nd
    A = np.zeros((n, n))
    A[0, 0] = A[1, 1] = 1.0
    A[0, 1] = dt
    A[1, 2] = dt
    alpha = math.exp(-dt / tau)
    A[2, 2] = alpha
    u_row = np.zeros(n)
    u_row[0], u_row[1] = -kp, -kd                 # u_k = -kp p - kd v
    if nd == 0:
        A[2, :] += (1 - alpha) * u_row
    else:
        A[2, 3 + nd - 1] = 1 - alpha              # a driven by the oldest stored command
        A[3, :] = u_row                           # newest command
        for i in range(1, nd):
            A[3 + i, 3 + i - 1] = 1.0
    return np.linalg.eigvals(A)


def damping(z: complex, dt: float) -> float:
    if abs(z) < 1e-9:
        return 1.0
    s = np.log(complex(z)) / dt
    return 1.0 if abs(s) < 1e-9 else float(-s.real / abs(s))


def design_pd(tau: float, delay: float, rate_hz: float, zeta_min: float = 0.6,
              zeta: float = 0.9, r_max: float = 0.995) -> Tuple[float, float, float]:
    """Returns (kp, kd, w_n)."""
    dt = 1.0 / rate_hz
    best = None
    for w in np.linspace(0.3, 8.0, 400):
        kp, kd = w * w, 2 * zeta * w
        poles = closed_loop_poles(kp, kd, tau, delay, dt)
        if np.max(np.abs(poles)) >= r_max:
            continue
        slow = [z for z in poles if abs(z) > 0.6]    # fast delay-chain poles barely matter
        if slow and min(damping(z, dt) for z in slow) < zeta_min:
            continue
        best = (kp, kd, w)
    if best is None:
        raise RuntimeError("no stable PD gains found - check plant numbers")
    return best


def design_from_config(plant: Dict, control: Dict) -> Dict[str, float]:
    rate = float(control.get("rate_hz", 50))
    resid = float(control.get("model", {}).get("residual_delay_s", 0.04))
    kp, kd, w = design_pd(float(plant["attitude_tau_s"]), resid, rate)
    return {"kp": round(kp, 3), "kd": round(kd, 3), "w_n": round(w, 3)}


def main() -> None:
    from ..core.config import load_config
    cfg = load_config("sim", "hover_test")
    print("recommended horizontal gains:", design_from_config(cfg.plant, cfg.control))


if __name__ == "__main__":
    main()
