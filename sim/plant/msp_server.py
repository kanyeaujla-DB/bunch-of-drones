"""SimDroneServer - makes one PlutoFirmwareModel reachable over MSP/TCP,
exactly like a real Pluto's Wi-Fi chip (which listens on TCP port 23).

Incoming packets are held back by link_delay_s (+ jitter) before the model
sees them, to imitate Wi-Fi latency. Uses the SAME msp_protocol module as the
real link, so bytes are identical in sim and on hardware.
"""

from __future__ import annotations

import asyncio
import heapq
import itertools
import logging
import struct
from typing import List, Optional, Tuple

import numpy as np

from pluto_swarm.core.clock import now
from pluto_swarm.hal import msp_protocol as msp

from .pluto_firmware_model import PlutoFirmwareModel

log = logging.getLogger(__name__)


class SimDroneServer:
    def __init__(self, drone_id: int, model: PlutoFirmwareModel, host: str, port: int,
                 delay_s: float = 0.0, jitter_s: float = 0.0,
                 rng: Optional[np.random.Generator] = None) -> None:
        self.drone_id = drone_id
        self.model = model
        self.host, self.port = host, port
        self.delay_s, self.jitter_s = delay_s, jitter_s
        self.rng = rng or np.random.default_rng()
        self._server: Optional[asyncio.AbstractServer] = None
        self._queue: List[Tuple[float, int, msp.Packet, asyncio.StreamWriter]] = []
        self._seq = itertools.count()
        self.clients = 0
        self._last_due = 0.0
        self._writers: set = set()

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._on_client, self.host, self.port)

    async def stop(self) -> None:
        """Like powering the drone off: drop every client, then stop listening.
        (Python >= 3.12 wait_closed() blocks until all client connections close.)"""
        if self._server:
            self._server.close()
            for w in list(self._writers):
                w.close()
            try:
                await asyncio.wait_for(self._server.wait_closed(), 2.0)
            except asyncio.TimeoutError:
                pass

    async def _on_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.clients += 1
        self._writers.add(writer)
        parser = msp.MspParser()
        try:
            while True:
                data = await reader.read(4096)
                if not data:
                    break
                for pkt in parser.feed(data):
                    if pkt.direction != msp.DIR_TO_DRONE:
                        continue
                    jitter = abs(self.rng.normal(0.0, self.jitter_s)) if self.jitter_s else 0.0
                    # TCP never reorders: a packet can't arrive before the previous one
                    due = max(now() + self.delay_s + jitter, self._last_due)
                    self._last_due = due
                    heapq.heappush(self._queue, (due, next(self._seq), pkt, writer))
        except (ConnectionError, OSError):
            pass
        finally:
            self.clients -= 1
            self._writers.discard(writer)
            writer.close()

    def process_due(self) -> None:
        """Called by the world's physics loop: deliver packets whose delay has passed."""
        t = now()
        while self._queue and self._queue[0][0] <= t:
            _, _, pkt, writer = heapq.heappop(self._queue)
            reply = self._handle(pkt)
            if reply and not writer.is_closing():
                try:
                    writer.write(reply)
                except (ConnectionError, OSError):
                    pass

    def _handle(self, pkt: msp.Packet) -> Optional[bytes]:
        m = self.model
        if pkt.cmd == msp.MSP_SET_RAW_RC:
            n = len(pkt.payload) // 2
            if n >= 8:
                m.on_rc(list(struct.unpack_from("<8H", pkt.payload)))
            return None
        if pkt.cmd == msp.MSP_SET_COMMAND:
            if pkt.payload:
                m.on_command(pkt.payload[0])
            return None
        if pkt.cmd == msp.MSP_ATTITUDE:
            return msp.encode_attitude_reply(*m.telemetry_attitude())
        if pkt.cmd == msp.MSP_ALTITUDE:
            return msp.encode_altitude_reply(*m.telemetry_altitude())
        if pkt.cmd == msp.MSP_ANALOG:
            v = m.telemetry_battery_v()
            return msp.encode_analog_reply(v, soc=int(100 * (v - 3.4) / 0.8))
        return None                       # unknown/unsupported: the real FC also stays quiet for many
