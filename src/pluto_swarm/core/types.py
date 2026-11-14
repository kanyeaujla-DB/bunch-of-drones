"""Shared data types - THE CONTRACT between all modules.

Every team's code meets here. Change a field only after the whole team agrees,
because localization, control, safety, sim and logging all depend on it.

Conventions (frozen):
  * World frame "arena": origin at a floor corner, X/Y along the walls, Z UP.
  * Body frame: x forward, y left, z up (ROS FLU convention).
  * Units: metres, seconds, radians (degrees only at the RC/telemetry boundary).
  * Time: seconds from time.monotonic() on the ground station.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np

RC_MIN = 1000
RC_MAX = 2000
RC_NEUTRAL = 1500


class Health(Enum):
    """How much we trust a drone's state estimate (architecture doc §4, step 6)."""
    OK = "ok"
    DEGRADED = "degraded"
    LOST = "lost"


class FirmwareCommand(Enum):
    """Values for MSP_SET_COMMAND (command.cpp in MagisV2). Sent ONCE, not repeated."""
    NONE = 0
    TAKE_OFF = 1
    LAND = 2


class FlightPhase(Enum):
    """Ground-station view of where each drone is in its flight."""
    IDLE = "idle"
    TAKING_OFF = "taking_off"
    FLYING = "flying"
    LANDING = "landing"
    LANDED = "landed"


@dataclass(frozen=True)
class DroneConfig:
    """One entry of config/swarm.yaml after the profile has been applied."""
    id: int
    host: str
    port: int
    marker_id: int
    start_slot: Tuple[float, float]


@dataclass
class RCFrame:
    """One MSP_SET_RAW_RC frame. Wire order: roll, pitch, throttle, yaw, aux1..aux4.

    Stick meaning in the firmware (to be verified at Airlink 1):
      roll  > 1500 -> roll right   (accelerate toward body -y)
      pitch > 1500 -> nose down    (accelerate toward body +x)
      throttle in alt-hold -> climb-rate request, 1500 = hold height
      yaw   > 1500 -> yaw rate (direction to verify)
    """
    roll: int = RC_NEUTRAL
    pitch: int = RC_NEUTRAL
    throttle: int = RC_NEUTRAL
    yaw: int = RC_NEUTRAL
    aux1: int = 1500
    aux2: int = 1000
    aux3: int = 1500
    aux4: int = 1000

    def as_list(self) -> List[int]:
        vals = [self.roll, self.pitch, self.throttle, self.yaw,
                self.aux1, self.aux2, self.aux3, self.aux4]
        return [int(min(RC_MAX, max(RC_MIN, round(v)))) for v in vals]

    @classmethod
    def from_list(cls, vals: List[int]) -> "RCFrame":
        vals = list(vals) + [1500] * (8 - len(vals))
        return cls(*vals[:8])


@dataclass
class Telemetry:
    """Latest values the drone reported. None = never received yet."""
    t: float = 0.0                       # ground-station time of last update (any field)
    roll_rad: Optional[float] = None     # + = right wing down
    pitch_rad: Optional[float] = None    # + = nose up (firmware convention, verify)
    yaw_rad: Optional[float] = None      # magnetometer heading
    alt_m: Optional[float] = None        # baro alt-hold estimate, relative to arming
    vario_m_s: Optional[float] = None    # vertical speed
    battery_v: Optional[float] = None
    t_attitude: float = 0.0
    t_altitude: float = 0.0
    t_analog: float = 0.0


@dataclass
class DroneState:
    """Estimated state of one drone, published by localization, read by control/safety."""
    id: int
    t: float                              # measurement (capture) time, not processing time
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))   # arena frame, m
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))   # m/s
    yaw: float = 0.0                      # rad, about +Z
    bias: np.ndarray = field(default_factory=lambda: np.zeros(3))  # estimated accel bias, m/s^2
    health: Health = Health.LOST
    source: str = "none"                  # "camera" | "sim_truth" | "fused" ...


@dataclass
class Setpoint:
    """What the planner wants one drone to do at time t (arena frame)."""
    id: int
    t: float
    pos: np.ndarray
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float = 0.0
