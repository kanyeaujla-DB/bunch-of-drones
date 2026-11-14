"""DroneLink - the swappable edge between our software and one drone.

Anything that can carry RC frames to a drone and telemetry back implements this:
  * MspLink          - MSP over TCP (real Pluto via Wi-Fi, AND our simulator).
                       The default: one asyncio loop drives every drone.
  * PlutoControlLink - FALLBACK: Drona's own PlutoControl library sends the
                       bytes (its writer thread); we only decode replies.

The rest of the code only ever talks to DroneLink, never to sockets.
"""

from __future__ import annotations

import abc
from typing import Optional

from ..core.clock import now
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

    # ---- optional hooks with safe defaults ----
    def write_tick(self) -> None:
        """Called by SwarmTicker once per tick. Links with their own sender
        thread (PlutoControlLink) leave this as a no-op."""

    async def drain(self) -> None:
        """Flush whatever write_tick queued."""

    def telemetry_age(self, t: Optional[float] = None) -> float:
        """Seconds since any telemetry arrived (inf if never)."""
        t = now() if t is None else t
        tel = self.telemetry
        return float("inf") if tel.t == 0.0 else t - tel.t
