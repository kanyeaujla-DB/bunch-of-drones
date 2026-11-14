"""PlutoControlLink - FALLBACK drone link built on Drona's PlutoControl library.

Select it with `link.type: plutocontrol` in a profile. Use it when:
  * Drona asks us to fly through their official library;
  * our MspLink misbehaves on some site and we need a known-good sender;
  * cross-checking: the same mission flown through both links must match.

How it works (no changes to PlutoControl itself):
  * PlutoControl's own writer thread streams RC at ~45 Hz and requests
    MSP_RC / ATTITUDE / RAW_IMU / ALTITUDE / ANALOG every tick - exactly as
    their library always does;
  * we set its rcRoll/rcPitch/... fields under its lock and use its own
    take_off()/land() sequences;
  * we do NOT call its blocking get_*() telemetry methods (they print, block and
    race the writer thread for the socket). A small reader thread decodes the
    replies its writer already requested, with our checksum-checked parser.

Trade-off vs MspLink: one sender thread per drone, so RC frames to different
drones are up to ~22 ms apart (MspLink: ~1 ms). Fine as a fallback.
"""

from __future__ import annotations

import asyncio
import logging
import threading
from typing import Dict, Optional

from ..core.clock import now
from ..core.types import DroneConfig, FirmwareCommand, RCFrame, Telemetry
from . import msp_protocol as msp
from .base import DroneLink

log = logging.getLogger(__name__)


def _import_pluto():
    try:
        from plutocontrol import Pluto   # PlutoControl >= 2.0.0
    except ImportError as e:              # pragma: no cover - depends on install
        raise RuntimeError("link.type 'plutocontrol' needs `pip install plutocontrol==2.0.0`") from e
    return Pluto


class PlutoControlLink(DroneLink):
    def __init__(self, drone: DroneConfig, control_cfg: Dict, pluto_cls=None) -> None:
        self.drone = drone
        self.drone_id = drone.id
        self._Pluto = pluto_cls or _import_pluto()
        self._p = None
        self._rc = RCFrame()
        self._tel = Telemetry()
        self._parser = msp.MspParser()
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()

    async def connect(self, timeout: float = 3.0, own_write_loop: bool = True) -> None:
        self._p = self._Pluto(ip=self.drone.host, port=self.drone.port)
        loop = asyncio.get_running_loop()
        await asyncio.wait_for(loop.run_in_executor(None, self._p.connect), timeout + 2.0)
        if not self._p.connected:
            raise ConnectionError(f"PlutoControl could not connect to {self.drone.host}:{self.drone.port}")
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_thread, daemon=True,
                                        name=f"pluto-reader-{self.drone_id}")
        self._reader.start()
        log.info("drone %d connected through PlutoControl (fallback link)", self.drone_id)

    async def close(self) -> None:
        self._stop.set()
        if self._p is not None:
            await asyncio.get_running_loop().run_in_executor(None, self._p.disconnect)

    def set_rc(self, frame: RCFrame) -> None:
        self._rc = frame
        p = self._p
        if p is None:
            return
        v = frame.as_list()
        with p._lock:                       # PlutoControl's own lock
            (p.rcRoll, p.rcPitch, p.rcThrottle, p.rcYaw,
             p.rcAUX1, p.rcAUX2, p.rcAUX3, p.rcAUX4) = v

    @property
    def rc(self) -> RCFrame:
        return self._rc

    def send_command(self, cmd: FirmwareCommand) -> None:
        p = self._p
        if p is None:
            return
        with p._lock:
            p.command_type = cmd.value      # their writer sends it once, then clears it

    @property
    def telemetry(self) -> Telemetry:
        return self._tel

    @property
    def connected(self) -> bool:
        return bool(self._p is not None and self._p.connected and
                    self._reader is not None and self._reader.is_alive())

    def _read_thread(self) -> None:
        while not self._stop.is_set() and self._p is not None and self._p.connected:
            try:
                data = self._p.client.recv(4096)
            except OSError:                  # includes socket.timeout
                continue
            if not data:
                log.error("drone %d: PlutoControl socket closed", self.drone_id)
                return
            for pkt in self._parser.feed(data):
                self._handle(pkt)

    def _handle(self, pkt: msp.Packet) -> None:
        if pkt.direction != msp.DIR_FROM_DRONE:
            return
        t, tel = now(), self._tel
        try:
            if pkt.cmd == msp.MSP_ATTITUDE:
                a = msp.decode_attitude(pkt.payload)
                tel.roll_rad, tel.pitch_rad, tel.yaw_rad, tel.t_attitude = (
                    a.roll_rad, a.pitch_rad, a.yaw_rad, t)
            elif pkt.cmd == msp.MSP_ALTITUDE:
                a = msp.decode_altitude(pkt.payload)
                tel.alt_m, tel.vario_m_s, tel.t_altitude = a.alt_m, a.vario_m_s, t
            elif pkt.cmd == msp.MSP_ANALOG:
                tel.battery_v, tel.t_analog = msp.decode_analog(pkt.payload).battery_v, t
            else:
                return
            tel.t = t
        except Exception as e:
            log.warning("drone %d: bad reply cmd %d: %s", self.drone_id, pkt.cmd, e)
