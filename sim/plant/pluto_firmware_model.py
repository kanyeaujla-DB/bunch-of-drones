"""PlutoFirmwareModel - imitates what MagisV2 firmware + airframe do with RC input.

This is NOT a generic quadcopter. It copies the firmware's own control modes,
so our controller sees the same "plant" in sim as on the real drone:

  roll/pitch stick -> tilt set-point (angle mode, 0.2 deg/us, clamp 20 deg)
                   -> first-order attitude lag -> acceleration g*tan(tilt)
  throttle stick   -> climb-rate request (alt-hold): (rc-1500)/4 cm/s,
                      +-40 us deadband, clamp -100..+120 cm/s
  TAKE_OFF command -> arm, climb to +1.2 m        (command.cpp)
  LAND command     -> descend, disarm on ground   (command.cpp)
  RC stream stops while armed -> firmware LANDs   (failsafe.cpp)
  AUX4 low         -> disarm (motors stop, drone falls)

Every number comes from config/plant/pluto_v5.yaml, which Airlink session 1
(system identification) will overwrite with measured values.

Frames: arena world X/Y/Z-up; body x forward, y left.
Internal tilt angles: phi = roll (+ = right wing down), theta = pitch (+ = nose DOWN,
i.e. accelerating forward). Telemetry reports pitch as + = nose UP (MultiWii style) -
VERIFY BOTH SIGNS AT AIRLINK 1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

import numpy as np


class Phase(Enum):
    DISARMED = "disarmed"
    ARMED_GROUND = "armed_ground"
    TAKING_OFF = "taking_off"
    FLYING = "flying"
    LANDING = "landing"


ARM_SWITCH_THRESHOLD = 1350     # AUX4 above this = arm switch ON (1500 arm, 1200 disarm)
MIN_CHECK = 1100                # Cleanflight arms from the switch only with throttle below this


@dataclass
class TruthState:
    pos: np.ndarray
    vel: np.ndarray
    phi: float = 0.0
    theta: float = 0.0
    yaw: float = 0.0


class PlutoFirmwareModel:
    def __init__(self, plant: Dict, start_xy=(0.0, 0.0), yaw: float = 0.0,
                 rng: Optional[np.random.Generator] = None) -> None:
        self.p = plant
        self.rng = rng or np.random.default_rng()
        self.g = float(plant.get("g", 9.81))
        self.s = TruthState(pos=np.array([start_xy[0], start_xy[1], 0.0]),
                            vel=np.zeros(3), yaw=yaw)
        self.phase = Phase.DISARMED
        self.rc: List[int] = [1500, 1500, 1500, 1500, 1500, 1000, 1500, 1000]
        self.t = 0.0
        self.t_last_rc: Optional[float] = None
        self.z_target = 0.0
        self.z_hold = 0.0
        self.z_at_arm = 0.0
        self.armed_time = 0.0
        self.baro_drift = 0.0
        sigma = float(plant.get("accel_bias_sigma_m_s2", 0.0))
        b = self.rng.normal(0.0, sigma, size=2)
        self.bias = np.array([b[0], b[1], 0.0])     # constant trim error (horizontal)
        self.events: List[str] = []
        self._switch_released = True    # firmware needs AUX4 low before it will re-arm

    # ---------------------------------------------------------------- inputs
    def on_rc(self, channels: List[int]) -> None:
        self.rc = list(channels[:8])
        self.t_last_rc = self.t
        # arm switch handling
        arm_switch = self.rc[7] > ARM_SWITCH_THRESHOLD
        if not arm_switch:
            self._switch_released = True
        if not arm_switch and self.phase != Phase.DISARMED:
            self._disarm("AUX4 low (operator disarm)")
        elif (arm_switch and self.phase == Phase.DISARMED and self.on_ground
              and self._switch_released and self.rc[2] < MIN_CHECK):
            self.phase = Phase.ARMED_GROUND
            self.z_at_arm = self.s.pos[2]
            self._switch_released = False
            self.events.append(f"{self.t:.2f} armed")

    def on_command(self, cmd: int) -> None:
        if self.phase == Phase.LANDING:
            return                                     # firmware ignores commands while landing
        if cmd == 1:                                   # TAKE_OFF
            # command.cpp takeOff(): arms the drone itself if the arm switch (BOXARM) is on.
            # That is why plutocontrol sends throttle 1800 + AUX4 1500 + TAKE_OFF.
            can_arm = (self.phase == Phase.DISARMED and self.on_ground and self._switch_released
                       and self.rc[7] > ARM_SWITCH_THRESHOLD)
            if self.phase == Phase.ARMED_GROUND or can_arm:
                if can_arm:
                    self.z_at_arm = self.s.pos[2]
                    self._switch_released = False
                    self.events.append(f"{self.t:.2f} armed by take-off command")
                self.phase = Phase.TAKING_OFF
                self.z_target = self.s.pos[2] + self.p["takeoff_height_cm"] / 100.0
                self.events.append(f"{self.t:.2f} take-off to {self.z_target:.2f} m")
        elif cmd == 2:                                 # LAND
            if self.phase in (Phase.TAKING_OFF, Phase.FLYING, Phase.ARMED_GROUND):
                self.phase = Phase.LANDING
                self.events.append(f"{self.t:.2f} landing")

    # ---------------------------------------------------------------- physics
    @property
    def on_ground(self) -> bool:
        return self.s.pos[2] <= 1e-3

    @property
    def armed(self) -> bool:
        return self.phase != Phase.DISARMED

    def _disarm(self, why: str) -> None:
        self.phase = Phase.DISARMED
        self.events.append(f"{self.t:.2f} disarmed ({why})")

    def _stick_climb_rate(self, throttle: int) -> float:
        p = self.p
        d = throttle - 1500
        if abs(d) <= p["alt_hold_deadband_us"]:
            return 0.0
        v_cm = d * p["alt_hold_cm_s_per_us"]
        v_cm = min(p["alt_hold_v_max_cm_s"], max(p["alt_hold_v_min_cm_s"], v_cm))
        return v_cm / 100.0

    def step(self, dt: float) -> None:
        p, s = self.p, self.s
        self.t += dt

        # failsafe: RC stream lost while armed -> land
        if (self.armed and self.phase != Phase.LANDING and self.t_last_rc is not None
                and self.t - self.t_last_rc > p["rc_loss_timeout_s"]):
            self.phase = Phase.LANDING
            self.events.append(f"{self.t:.2f} RC lost -> failsafe land")

        if self.armed:
            self.armed_time += dt

        # ---- vertical: firmware alt-hold turns everything into a climb-rate request
        if self.phase == Phase.TAKING_OFF:
            # slow down near the target like the firmware's position-hold loop
            vz_cmd = min(p["takeoff_climb_cm_s"] / 100.0, 2.0 * (self.z_target - s.pos[2]))
            if s.pos[2] >= self.z_target - 0.02:
                self.phase = Phase.FLYING
                self.z_hold = self.z_target
                self.events.append(f"{self.t:.2f} take-off complete")
        elif self.phase == Phase.LANDING:
            vz_cmd = -p["land_descent_cm_s"] / 100.0
        elif self.phase == Phase.FLYING:
            vz_cmd = self._stick_climb_rate(self.rc[2])
            if vz_cmd == 0.0:          # stick centred: alt-hold holds the remembered height
                vz_cmd = 2.0 * (self.z_hold - s.pos[2])
            else:
                self.z_hold = s.pos[2]
        elif self.phase == Phase.ARMED_GROUND:
            vz_cmd = max(0.0, self._stick_climb_rate(self.rc[2]))
            if vz_cmd > 0:
                self.phase = Phase.FLYING
                self.z_hold = s.pos[2]
        else:
            vz_cmd = None                               # disarmed: no thrust

        if vz_cmd is None:
            s.vel[2] -= self.g * dt                     # falling
        else:
            s.vel[2] += (vz_cmd - s.vel[2]) * min(1.0, dt / p["z_tau_s"])

        # ---- horizontal: angle mode
        airborne_ctrl = self.phase in (Phase.TAKING_OFF, Phase.FLYING, Phase.LANDING) and not self.on_ground
        if airborne_ctrl:
            k = p["stick_to_tilt_deg_per_us"]
            lim = p["max_tilt_deg"]
            phi_sp = math.radians(max(-lim, min(lim, (self.rc[0] - 1500) * k)))
            theta_sp = math.radians(max(-lim, min(lim, (self.rc[1] - 1500) * k)))
            a = min(1.0, dt / p["attitude_tau_s"])
            s.phi += (phi_sp - s.phi) * a
            s.theta += (theta_sp - s.theta) * a
            ax_b = self.g * math.tan(s.theta)           # nose down -> forward
            ay_b = -self.g * math.tan(s.phi)            # right wing down -> toward -y (right)
            c, sn = math.cos(s.yaw), math.sin(s.yaw)
            acc = np.array([c * ax_b - sn * ay_b, sn * ax_b + c * ay_b])
            acc += -p["drag_per_s"] * s.vel[:2] + self.bias[:2]
            s.vel[:2] += acc * dt
            yaw_rate = math.radians((self.rc[3] - 1500) * p["yaw_rate_deg_s_per_us"])
            s.yaw = (s.yaw + yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        else:
            s.phi = s.theta = 0.0
            if self.on_ground:
                s.vel[:2] = 0.0

        s.pos += s.vel * dt

        # ---- ground contact
        if s.pos[2] <= 0.0:
            s.pos[2] = 0.0
            if s.vel[2] < 0:
                s.vel[2] = 0.0
            s.vel[:2] = 0.0
            if self.phase == Phase.LANDING:
                self._disarm("landed")

        self.baro_drift += p["baro_drift_cm_per_min"] / 100.0 / 60.0 * dt * self.rng.choice([-1, 1])

    # ---------------------------------------------------------------- telemetry
    def telemetry_attitude(self):
        return self.s.phi, -self.s.theta, self.s.yaw   # roll, pitch(+nose up), yaw

    def telemetry_altitude(self):
        noise = self.rng.normal(0.0, self.p["baro_noise_cm"] / 100.0)
        return self.s.pos[2] - self.z_at_arm + noise + self.baro_drift, self.s.vel[2]

    def telemetry_battery_v(self) -> float:
        return self.p["battery_full_v"] - self.p["battery_drain_v_per_min"] * self.armed_time / 60.0
