"""SwarmManager - owns all drone links and the flight state machine.

P0 scope: connect -> preflight -> take off all -> hover -> land all.
P1+ adds: the per-tick position loop (localization -> planning -> control -> safety)
inside SwarmTicker._loop, between "read state" and "write RC".

Operator safety (architecture doc §7, layer 3):
    Ctrl+C once  -> LAND all drones
    Ctrl+C twice -> DISARM all (motors stop - drones drop! last resort)
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import replace
from typing import Callable, Dict, List, Optional

from ..core.clock import now
from ..core.config import Config
from ..core.types import FirmwareCommand, FlightPhase, RCFrame
from ..hal.msp_link import MspLink

log = logging.getLogger(__name__)


class SwarmTicker:
    """ONE loop sends every drone's RC frame in the same tick (sync to ~1 ms).
    on_tick(t) runs first each tick - that is where the control loop will live."""

    def __init__(self, links: List[MspLink], rate_hz: float,
                 on_tick: Optional[Callable[[float], None]] = None) -> None:
        self.links, self.rate_hz, self.on_tick = links, rate_hz, on_tick
        self._task: Optional[asyncio.Task] = None
        self.ticks = 0
        self.late_ticks = 0

    def start(self) -> None:
        self._task = asyncio.ensure_future(self._loop())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def _loop(self) -> None:
        period = 1.0 / self.rate_hz
        next_t = now()
        while True:
            t = now()
            if self.on_tick:
                self.on_tick(t)
            for link in self.links:            # all frames written back-to-back
                link.write_tick()
            await asyncio.gather(*(l.drain() for l in self.links))
            self.ticks += 1
            next_t += period
            delay = next_t - now()
            if delay < 0:
                self.late_ticks += 1
                if delay < -period:
                    next_t = now()
                delay = 0.0
            await asyncio.sleep(delay)


class SwarmManager:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.rc_cfg = cfg.control.get("rc", {})
        self.links: Dict[int, MspLink] = {d.id: MspLink(d, cfg.control) for d in cfg.drones}
        self.phase: Dict[int, FlightPhase] = {i: FlightPhase.IDLE for i in self.links}
        self.ticker = SwarmTicker(list(self.links.values()), cfg.control.get("rate_hz", 50))
        self.abort_requested = False

    # ------------------------------------------------------------ connection
    async def connect_all(self) -> List[str]:
        timeout = float(self.cfg.safety.get("connect_timeout_s", 3.0))
        results = await asyncio.gather(
            *(l.connect(timeout, own_write_loop=False) for l in self.links.values()),
            return_exceptions=True)
        problems = [f"drone {i}: cannot connect to {l.drone.host}:{l.drone.port} ({r!r})"
                    for (i, l), r in zip(self.links.items(), results) if isinstance(r, Exception)]
        if not problems:
            self.ticker.start()
        return problems

    async def close_all(self) -> None:
        await self.ticker.stop()
        await asyncio.gather(*(l.close() for l in self.links.values()))

    # ------------------------------------------------------------ preflight
    async def preflight(self, wait_s: float = 3.0) -> List[str]:
        """Layer-1 hardware checks. Empty list = OK to arm."""
        t_end = now() + wait_s
        while now() < t_end:
            if all(l.telemetry.battery_v is not None and l.telemetry.alt_m is not None
                   and l.telemetry.roll_rad is not None for l in self.links.values()):
                break
            await asyncio.sleep(0.1)
        problems = []
        vmin = float(self.cfg.safety.get("min_battery_v_arm", 3.7))
        for i, l in self.links.items():
            tel = l.telemetry
            if not l.connected:
                problems.append(f"drone {i}: not connected")
            elif tel.battery_v is None or tel.alt_m is None or tel.roll_rad is None:
                problems.append(f"drone {i}: no telemetry received")
            elif tel.battery_v < vmin:
                problems.append(f"drone {i}: battery {tel.battery_v:.2f} V < {vmin} V")
        return problems

    # ------------------------------------------------------------ RC helpers
    def _frame(self, armed: bool, throttle: int = 1500) -> RCFrame:
        rc = self.rc_cfg
        return RCFrame(throttle=throttle, aux1=rc.get("aux1", 1500), aux2=rc.get("aux2", 1000),
                       aux3=rc.get("aux3", 1500),
                       aux4=rc.get("aux4_armed", 1500) if armed else rc.get("aux4_disarm_cmd", 1200))

    def set_all(self, frame: RCFrame) -> None:
        for l in self.links.values():
            l.set_rc(replace(frame))

    # ------------------------------------------------------------ flight
    async def takeoff_all(self, timeout_s: float = 8.0) -> bool:
        log.info("TAKE-OFF all drones")
        self.set_all(self._frame(armed=True, throttle=int(self.rc_cfg.get("takeoff_throttle", 1800))))
        for i, l in self.links.items():
            l.send_command(FirmwareCommand.TAKE_OFF)
            self.phase[i] = FlightPhase.TAKING_OFF
        await asyncio.sleep(float(self.rc_cfg.get("takeoff_hold_s", 0.5)))
        self.set_all(self._frame(armed=True, throttle=1500))     # 1500 = hold height

        target = 0.8 * self.cfg.plant.get("takeoff_height_cm", 120) / 100.0
        t_end = now() + timeout_s
        while now() < t_end and not self.abort_requested:
            if all((l.telemetry.alt_m or 0.0) >= target for l in self.links.values()):
                for i in self.links:
                    self.phase[i] = FlightPhase.FLYING
                return True
            await asyncio.sleep(0.05)
        return False

    async def hover(self, seconds: float) -> None:
        log.info("HOVER %.1f s", seconds)
        t_end = now() + seconds
        while now() < t_end and not self.abort_requested:
            self._check_links()
            await asyncio.sleep(0.05)

    async def land_all(self, timeout_s: float = 20.0) -> bool:
        log.info("LAND all drones")
        self.set_all(self._frame(armed=True, throttle=1500))   # keep arm switch ON while landing
        for i, l in self.links.items():
            l.send_command(FirmwareCommand.LAND)
            self.phase[i] = FlightPhase.LANDING
        await asyncio.sleep(1.0)
        t_end = now() + timeout_s
        while now() < t_end:
            if all(self._looks_landed(l) for l in self.links.values()):
                break
            await asyncio.sleep(0.1)
        ok = all(self._looks_landed(l) for l in self.links.values())
        self.disarm_all()
        return ok

    def disarm_all(self) -> None:
        log.warning("DISARM all drones")
        self.set_all(self._frame(armed=False, throttle=1000))
        for i in self.links:
            self.phase[i] = FlightPhase.LANDED

    @staticmethod
    def _looks_landed(l: MspLink) -> bool:
        tel = l.telemetry
        return (tel.alt_m is not None and tel.alt_m < 0.15
                and abs(tel.vario_m_s or 0.0) < 0.05)

    def _check_links(self) -> None:
        timeout = float(self.cfg.safety.get("telemetry_timeout_s", 0.5))
        t = now()
        for i, l in self.links.items():
            if not l.connected or t - l.telemetry.t > timeout:
                log.error("drone %d: link/telemetry lost - firmware will land it; aborting mission", i)
                self.abort_requested = True
