"""DroneLink - the swappable edge between our software and one drone.

Anything that can carry RC frames to a drone and telemetry back implements this:
  * MspLink    - MSP over TCP (real Pluto via Wi-Fi, AND our simulator)
  * (later)    - an in-process link for very fast tests, if ever needed

The rest of the code only ever talks to DroneLink, never to sockets.
"""

from __future__ import annotations

import abc

from ..core.types import FirmwareCommand, RCFrame, Telemetry


class DroneLink(abc.ABC):
    drone_id: int

    @abc.abstractmethod
    async def connect(self, timeout: float) -> None:
        """Open the connection and start streaming RC + polling telemetry."""

    @abc.abstractmethod
    async def close(self) -> None:
        """Stop streaming and close. NOTE: the firmware lands a drone whose RC
        stream stops while armed - closing is not a safe way to stop a flight."""

    @abc.abstractmethod
    def set_rc(self, frame: RCFrame) -> None:
        """Set the RC frame that is streamed every tick until changed."""

    @abc.abstractmethod
    def send_command(self, cmd: FirmwareCommand) -> None:
        """Queue a one-shot firmware command (take-off / land)."""

    @property
    @abc.abstractmethod
    def telemetry(self) -> Telemetry:
        """Latest telemetry received."""

    @property
    @abc.abstractmethod
    def connected(self) -> bool:
        ...
