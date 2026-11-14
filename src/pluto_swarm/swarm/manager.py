"""SwarmManager - owns all drone links and the flight state machine.

Open loop (P0, no camera): connect -> preflight -> take off all -> hover -> land all.
Closed loop (fly_mission): the per-tick loop in swarm/flight.py runs inside
SwarmTicker, between "read state" and "write RC":
    preflight (links + tracking) -> take-off with XY hold -> plan + plan
    validation -> TRANSFER -> FORMATION -> SETTLE -> land with XY hold -> disarm.

Arm / take-off / land copy PlutoControl 2.0.0 exactly:
    take_off(): disarm frame, then throttle 1800 + AUX4 1500 + MSP_SET_COMMAND 1
    land():     MSP_SET_COMMAND 2 (sticks unchanged)
    disarm():   throttle 1300 + AUX4 1200

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
from ..core.types import FirmwareCommand, FlightPhase, Health, RCFrame
from ..hal.base import DroneLink
from ..hal.factory import make_link

log = logging.getLogger(__name__)


class SwarmTicker:
    """ONE loop sends every drone's RC frame in the same tick (sync to ~1 ms).
    on_tick(t) runs first each tick - that is where the control loop will live."""

    def __init__(self, links: List[DroneLink], rate_hz: float,
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
        self.links: Dict[int, DroneLink] = {d.id: make_link(d, cfg.profile, cfg.control)
                                            for d in cfg.drones}
        self.phase: Dict[int, FlightPhase] = {i: FlightPhase.IDLE for i in self.links}
        self.ticker = SwarmTicker(list(self.links.values()), cfg.control.get("rate_hz", 50))
        self.abort_requested = False
        self.base: Dict[int, RCFrame] = {i: self._frame(armed=False, throttle=1000) for i in self.links}
        for i, l in self.links.items():
            self.base[i] = replace(self.base[i], aux4=self.rc_cfg.get("aux4_disarmed", 1000))
        self.flight = None                       # ClosedLoopFlight when a camera is available

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
        """Base frame for every drone (arm switch, throttle during take-off/land).
        With closed-loop flight attached, it overlays roll/pitch/yaw(/throttle)."""
        for i, l in self.links.items():
            self.base[i] = replace(frame)
            l.set_rc(replace(frame))

    def set_one(self, i: int, frame: RCFrame) -> None:
        self.base[i] = replace(frame)
        self.links[i].set_rc(replace(frame))

    def base_frame(self, i: int) -> RCFrame:
        return self.base[i]

    def _airborne(self, i: int, value: bool) -> None:
        if self.flight is not None:
            self.flight.airborne[i] = value

    # ------------------------------------------------------------ flight
    async def takeoff_all(self, timeout_s: float = 8.0) -> bool:
        log.info("TAKE-OFF all drones")
        # PlutoControl take_off(): disarm frame first (arm switch must be seen LOW),
        # then box-arm (throttle 1800, AUX4 1500) together with TAKE_OFF.
        self.set_all(self._frame(armed=False, throttle=1300))
        await asyncio.sleep(2.5 / float(self.cfg.control.get("rate_hz", 50)))
        self.set_all(self._frame(armed=True, throttle=int(self.rc_cfg.get("takeoff_throttle", 1800))))
        for i, l in self.links.items():
            l.send_command(FirmwareCommand.TAKE_OFF)
            self.phase[i] = FlightPhase.TAKING_OFF
            self._airborne(i, True)
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
            if self.flight is not None and self.flight.mode[i].value in ("full", "xy_hold"):
                from .flight import Mode
                self.flight.hold_here(i, Mode.XY_HOLD)          # keep XY while descending
            if not (self.flight is not None and self.flight.land_requested[i]):
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
        if self.flight is not None:
            from .flight import Mode
            for i in self.links:
                self.flight.mode[i] = Mode.OFF
        self.set_all(self._frame(armed=False, throttle=int(self.rc_cfg.get("disarm_throttle", 1300))))
        for i in self.links:
            self.phase[i] = FlightPhase.LANDED
            self._airborne(i, False)

    @staticmethod
    def _looks_landed(l: DroneLink) -> bool:
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

    # ------------------------------------------------------------ closed loop
    def attach_flight(self, localizer) -> None:
        from .flight import ClosedLoopFlight
        self.flight = ClosedLoopFlight(self.cfg, self.links, localizer, self.base_frame)
        self.ticker.on_tick = self.flight.on_tick

    async def wait_tracking(self, timeout_s: float = 5.0) -> List[str]:
        """Every drone must be seen by the camera (health OK) before arming."""
        loc = self.flight.loc
        t_end = now() + timeout_s
        while now() < t_end:
            if all(loc.state(i).health == Health.OK for i in self.links):
                return []
            await asyncio.sleep(0.1)
        return [f"drone {i}: not tracked by the camera ({loc.state(i).health.value})"
                for i in self.links if loc.state(i).health != Health.OK]

    async def fly_mission(self, mission: Dict, limits: Dict) -> int:
        """Closed-loop mission. Returns 0 on success, non-zero on abort."""
        from ..planning.mission import MissionPlan
        from ..safety.preflight_validation import validate_plan
        from .flight import Mode
        fl = self.flight
        for i in self.links:                   # XY hold over the take-off spot
            fl.hold_here(i, Mode.XY_HOLD)
        ok = await self.takeoff_all()
        if not ok or self.abort_requested:
            log.error("take-off failed or aborted")
            return 4
        start = {i: fl.loc.state(i).pos for i in self.links}
        plan = MissionPlan(mission, start, limits)
        rep = validate_plan(plan, self.cfg.arena, self.cfg.safety, self.cfg.plant, self.cfg.control)
        self.plan_report = rep
        log.info("plan: transfer %.1f s, formation %.1f s, min separation %.3f m, max speed %.2f m/s",
                 plan.t_transfer, plan.formation_duration, rep.min_separation_m, rep.max_speed_m_s)
        if not rep.ok:
            for p in rep.problems:
                log.error("PLAN REJECTED: %s", p)
            return 6
        fl.start_mission(plan, now())
        while not fl.mission_done and not self.abort_requested:
            self._check_links()
            await asyncio.sleep(0.05)
        if fl.abort_reason or self.abort_requested:
            for i in self.links:                # survivors hold briefly, then everyone lands
                if fl.mode[i] == Mode.FULL:
                    fl.hold_here(i, Mode.FULL)
            await asyncio.sleep(float(self.cfg.safety.get("abort_hold_s", 1.0)))
            return 7
        return 0
