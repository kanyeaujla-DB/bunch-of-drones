"""MspLink - async MSP-over-TCP link to ONE drone (real Pluto or simulator).

Why asyncio and not threads: one event loop can drive 4-10 sockets in the same
tick, so RC frames to all drones leave within ~1-2 ms of each other (sync!), and
there is no thread-scheduling latency (the e-Yantra paper measured ~341 ms).

Each tick (rate_hz from config/control.yaml) the link sends:
  1. MSP_SET_RAW_RC with the current RCFrame   (must stream continuously -
     the firmware lands the drone if RC stops while armed)
  2. any queued one-shot command (take-off / land)
  3. telemetry requests (attitude, altitude every tick; battery at 1 Hz)
Replies are parsed in a separate reader task and stored in .telemetry.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Dict, List, Optional

from ..core.clock import now
from ..core.types import DroneConfig, FirmwareCommand, RCFrame, Telemetry
from . import msp_protocol as msp
from .base import DroneLink

log = logging.getLogger(__name__)


class MspLink(DroneLink):
    def __init__(self, drone: DroneConfig, control_cfg: Dict) -> None:
        self.drone = drone
        self.drone_id = drone.id
        self.rate_hz = float(control_cfg.get("rate_hz", 50))
        tel = control_cfg.get("telemetry", {})
        self._every_att = int(tel.get("attitude_every_n_ticks", 1))
        self._every_alt = int(tel.get("altitude_every_n_ticks", 1))
        self._every_ana = int(tel.get("analog_every_n_ticks", 50))
        rc = control_cfg.get("rc", {})
        self._rc = RCFrame(aux1=rc.get("aux1", 1500), aux2=rc.get("aux2", 1000),
                           aux3=rc.get("aux3", 1500), aux4=rc.get("aux4_disarmed", 1000))
        self._cmd_queue: List[FirmwareCommand] = []
        self._tel = Telemetry()
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._tasks: List[asyncio.Task] = []
        self._connected = False
        self._tick = 0
        self.frames_sent = 0
        self.parser = msp.MspParser()

    # ---- DroneLink API ----
    async def connect(self, timeout: float = 3.0, own_write_loop: bool = True) -> None:
        """own_write_loop=False: the caller (SwarmTicker) calls write_tick() for all
        drones in one loop, so every drone's frame leaves in the same tick."""
        self._reader, self._writer = await asyncio.wait_for(
            asyncio.open_connection(self.drone.host, self.drone.port), timeout)
        self._connected = True
        log.info("drone %d connected to %s:%d", self.drone_id, self.drone.host, self.drone.port)
        self._tasks = [asyncio.ensure_future(self._read_loop())]
        if own_write_loop:
            self._tasks.append(asyncio.ensure_future(self._write_loop()))

    def write_tick(self) -> None:
        """Queue one tick of bytes on the socket (non-blocking). Used by SwarmTicker."""
        if not self._connected:
            return
        try:
            self._writer.write(self.build_tick_bytes())
            self.frames_sent += 1
        except (ConnectionError, OSError) as e:
            log.error("drone %d write failed: %s", self.drone_id, e)
            self._connected = False

    async def drain(self) -> None:
        if self._connected:
            try:
                await self._writer.drain()
            except (ConnectionError, OSError) as e:
                log.error("drone %d drain failed: %s", self.drone_id, e)
                self._connected = False

    async def close(self) -> None:
        self._connected = False
        for t in self._tasks:
            t.cancel()
        for t in self._tasks:
            try:
                await t
            except (asyncio.CancelledError, Exception):
                pass
        if self._writer is not None:
            self._writer.close()
            try:
                await self._writer.wait_closed()
            except Exception:
                pass

    def set_rc(self, frame: RCFrame) -> None:
        self._rc = frame

    @property
    def rc(self) -> RCFrame:
        return self._rc

    def send_command(self, cmd: FirmwareCommand) -> None:
        self._cmd_queue.append(cmd)

    @property
    def telemetry(self) -> Telemetry:
        return self._tel

    @property
    def connected(self) -> bool:
        return self._connected

    # ---- internals ----
    def build_tick_bytes(self) -> bytes:
        """All bytes for one tick. Public so the drone-sync scheduler and tests can use it."""
        out = bytearray(msp.encode_set_raw_rc(self._rc.as_list()))
        while self._cmd_queue:
            out += msp.encode_set_command(self._cmd_queue.pop(0).value)
        if self._tick % self._every_att == 0:
            out += msp.encode_request(msp.MSP_ATTITUDE)
        if self._tick % self._every_alt == 0:
            out += msp.encode_request(msp.MSP_ALTITUDE)
        if self._tick % self._every_ana == 0:
            out += msp.encode_request(msp.MSP_ANALOG)
        self._tick += 1
        return bytes(out)

    async def _write_loop(self) -> None:
        period = 1.0 / self.rate_hz
        next_t = now()
        try:
            while self._connected:
                self._writer.write(self.build_tick_bytes())
                await self._writer.drain()
                self.frames_sent += 1
                next_t += period
                delay = next_t - now()
                if delay < -period:          # fell far behind: resync, don't burst
                    next_t = now()
                    delay = 0
                await asyncio.sleep(max(0.0, delay))
        except (ConnectionError, OSError) as e:
            log.error("drone %d write failed: %s", self.drone_id, e)
            self._connected = False

    async def _read_loop(self) -> None:
        try:
            while self._connected:
                data = await self._reader.read(4096)
                if not data:
                    log.error("drone %d: connection closed by drone", self.drone_id)
                    self._connected = False
                    return
                for pkt in self.parser.feed(data):
                    self._handle(pkt)
        except (ConnectionError, OSError) as e:
            log.error("drone %d read failed: %s", self.drone_id, e)
            self._connected = False

    def _handle(self, pkt: msp.Packet) -> None:
        if pkt.direction != msp.DIR_FROM_DRONE:
            return
        t = now()
        tel = self._tel
        try:
            if pkt.cmd == msp.MSP_ATTITUDE:
                a = msp.decode_attitude(pkt.payload)
                tel.roll_rad, tel.pitch_rad, tel.yaw_rad = a.roll_rad, a.pitch_rad, a.yaw_rad
                tel.t_attitude = t
            elif pkt.cmd == msp.MSP_ALTITUDE:
                a = msp.decode_altitude(pkt.payload)
                tel.alt_m, tel.vario_m_s = a.alt_m, a.vario_m_s
                tel.t_altitude = t
            elif pkt.cmd == msp.MSP_ANALOG:
                tel.battery_v = msp.decode_analog(pkt.payload).battery_v
                tel.t_analog = t
            else:
                return
            tel.t = t
        except Exception as e:     # a malformed reply must never kill the link
            log.warning("drone %d: bad reply cmd %d: %s", self.drone_id, pkt.cmd, e)
