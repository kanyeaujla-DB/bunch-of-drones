"""ClosedLoopFlight - the per-tick loop that turns the swarm into a controlled
formation (architecture doc, "At a glance" diagram). Runs inside SwarmTicker, so
for ALL drones, in one tick:

  1. actuator model -> what each drone is doing with the sticks we sent
  2. localizer      -> baro fused, filter predicted to now, health
  3. mission        -> references from ONE clock (formation feedback slows it)
  4. watchdog       -> CONTINUE / HOLD / LAND per drone, abort policy
  5. controller     -> desired accelerations on the state predicted ahead by
                       the command delay (Smith-predictor style)
  6. safety filter  -> CBF keeps every pair separated (downwash ellipsoid)
  7. RC mapper      -> integer sticks (+ deadband dither), streamed this tick

Per-drone modes
  OFF      sticks from the manager's base frame (arming, disarm, emergencies)
  XY_HOLD  closed-loop roll/pitch/yaw to a target; throttle from the base frame
           (used during firmware take-off and landing, so XY never drifts)
  FULL     closed-loop in all axes (mission)
  NEUTRAL  level sticks, throttle 1500 - firmware holds attitude + height
           (tracking degraded: don't steer on a stale estimate)
"""

from __future__ import annotations

import logging
from dataclasses import replace
from enum import Enum
from typing import Callable, Dict, Optional

import numpy as np

from ..control.actuator import ActuatorModel
from ..control.position_controller import PositionController
from ..control.rc_mapper import RCMapper
from ..core.config import Config
from ..core.types import FirmwareCommand, Health, RCFrame, Setpoint
from ..localization.localizer import Localizer
from ..planning.mission import MissionPlan, Phase
from ..safety.cbf_filter import SafetyFilter
from ..safety.geofence import Geofence
from ..safety.watchdog import Verdict, Watchdog
from .synchronizer import FormationClock

log = logging.getLogger(__name__)


class Mode(Enum):
    OFF = "off"
    XY_HOLD = "xy_hold"
    FULL = "full"
    NEUTRAL = "neutral"


class ClosedLoopFlight:
    def __init__(self, cfg: Config, links: Dict, localizer: Localizer,
                 base_frame: Callable[[int], RCFrame]) -> None:
        self.cfg = cfg
        self.links = links
        self.loc = localizer
        self.base_frame = base_frame
        ids = list(links)
        self.ids = ids
        self.ctrl = {i: PositionController(cfg.plant, cfg.control) for i in ids}
        self.act = {i: ActuatorModel(cfg.plant, cfg.control) for i in ids}
        self.mapper = RCMapper(cfg.plant, cfg.control)
        self.cbf = SafetyFilter(cfg.safety)
        self.fence = Geofence(cfg.arena)
        self.watch = Watchdog(cfg.safety, cfg.arena)
        self.clock = FormationClock(cfg.control.get("sync", {}))
        model = cfg.control.get("model", {})
        self.predict_ahead = float(model.get("command_delay_s", 0.05)) + float(model.get("predict_extra_s", 0.0))
        # feed-forward lead: the drone answers a tilt command after delay + attitude lag,
        # a climb-rate command after the delay - so take those terms from slightly AHEAD
        ff = cfg.control.get("feedforward", {})
        self.lead_xy = float(ff.get("lead_xy_s", float(model.get("command_delay_s", 0.05))
                                    + float(cfg.plant["attitude_tau_s"])))
        self.lead_z = float(ff.get("lead_z_s", float(model.get("command_delay_s", 0.05))))
        self.mode: Dict[int, Mode] = {i: Mode.OFF for i in ids}
        self._resume: Dict[int, Mode] = {i: Mode.OFF for i in ids}   # mode to restore after NEUTRAL
        self.target: Dict[int, Optional[np.ndarray]] = {i: None for i in ids}
        self.airborne: Dict[int, bool] = {i: False for i in ids}
        self.land_requested: Dict[int, bool] = {i: False for i in ids}
        self.plan: Optional[MissionPlan] = None
        self.phase = Phase.DONE
        self._t_phase0 = 0.0
        self.tau = 0.0
        self.abort_reason: Optional[str] = None
        self._t_last: Optional[float] = None
        self.last: Dict[int, dict] = {}            # per-drone snapshot for logging
        self.ticks = 0
        self.errors = 0
        self.degraded_events = 0

    # ------------------------------------------------------------ commands from the manager
    def hold_here(self, did: int, mode: Mode = Mode.XY_HOLD) -> None:
        st = self.loc.state(did)
        self.target[did] = st.pos.copy()
        self.mode[did] = mode
        self.ctrl[did].reset()

    def hold_at(self, did: int, pos: np.ndarray, mode: Mode = Mode.XY_HOLD) -> None:
        self.target[did] = np.asarray(pos, float).copy()
        self.mode[did] = mode
        self.ctrl[did].reset()

    def start_mission(self, plan: MissionPlan, t: float) -> None:
        self.plan = plan
        self.phase = Phase.TRANSFER
        self._t_phase0 = t
        self.tau = 0.0
        for i in self.ids:
            self.mode[i] = Mode.FULL
            self.ctrl[i].reset()

    def stop_mission(self, reason: str) -> None:
        """Abort: every controllable drone holds where it is (closed loop)."""
        if self.abort_reason is None:
            self.abort_reason = reason
            log.error("MISSION ABORT: %s", reason)
        self.phase = Phase.DONE
        for i in self.ids:
            if self.mode[i] == Mode.FULL:
                self.hold_here(i, Mode.FULL)

    @property
    def mission_done(self) -> bool:
        return self.phase == Phase.DONE

    # ------------------------------------------------------------ the tick
    def on_tick(self, t: float) -> None:
        try:
            self._tick(t)
        except Exception:                          # never let control kill the RC stream
            self.errors += 1
            log.exception("control tick failed - neutral sticks, mission aborted")
            for i in self.ids:
                if self.mode[i] != Mode.OFF:
                    self.mode[i] = Mode.NEUTRAL
                self.links[i].set_rc(self._compose(i, None))
            self.abort_reason = self.abort_reason or "control exception"
            self.phase = Phase.DONE

    def _tick(self, t: float) -> None:
        dt = 0.02 if self._t_last is None else max(1e-3, t - self._t_last)
        self._t_last = t
        self.ticks += 1

        # 1-2: actuator model -> filter inputs; baro; predict
        inputs = {}
        for i in self.ids:
            yaw = self.loc.est[i].yaw
            inputs[i] = self.act[i].input_at(t, yaw)
            link = self.links[i]
            self.loc.on_telemetry(i, link.telemetry, getattr(link, "rtt_s", None))
        self.loc.predict(t, inputs)
        states = {i: self.loc.state(i, t) for i in self.ids}

        # 3: references
        refs = self._references(t, states)

        # 4: watchdog
        for i in self.ids:
            if self.mode[i] == Mode.OFF and not self.airborne[i]:
                continue
            chk = self.watch.check(t, i, states[i].health, self.links[i], states[i].pos,
                                   refs[i].pos if i in refs and self.mode[i] == Mode.FULL else None,
                                   self.airborne[i])
            if chk.verdict == Verdict.LAND and not self.land_requested[i]:
                self.land_requested[i] = True
                msg = f"drone {i}: LAND - {', '.join(chk.reasons)}"
                self.watch.events.append(msg)
                log.error(msg)
                self.links[i].send_command(FirmwareCommand.LAND)
                self.mode[i] = Mode.NEUTRAL if states[i].health != Health.OK else Mode.XY_HOLD
                if self.mode[i] == Mode.XY_HOLD:
                    self.target[i] = states[i].pos.copy()
                if not self.mission_done:
                    self.stop_mission(msg)
            elif chk.verdict == Verdict.HOLD and self.mode[i] in (Mode.FULL, Mode.XY_HOLD):
                self._resume[i] = self.mode[i]
                self.mode[i] = Mode.NEUTRAL
                self.degraded_events += 1
                self.watch.events.append(f"t={t:.2f} drone {i}: tracking degraded -> neutral hold")
            elif chk.verdict == Verdict.CONTINUE and self.mode[i] == Mode.NEUTRAL \
                    and not self.land_requested[i]:
                self.mode[i] = self._resume[i]             # back to what it was doing

        # 5: controller (on the state predicted ahead by the command delay)
        u_hat, outs = {}, {}
        for i in self.ids:
            if self.mode[i] not in (Mode.FULL, Mode.XY_HOLD) or i not in refs:
                continue
            p, v = self.loc.peek_ahead(i, self.predict_ahead)
            out = self.ctrl[i].compute(p, v, states[i].bias, refs[i], dt,
                                       xy_enabled=True)
            outs[i] = out
            u_hat[i] = np.array([out.a_xy[0], out.a_xy[1], out.az])

        # 6: safety filter over every controlled drone
        if len(u_hat) >= 2:
            pos = {i: states[i].pos for i in u_hat}
            vel = {i: states[i].vel for i in u_hat}
            u_safe = self.cbf.filter(pos, vel, u_hat)
        else:
            u_safe = u_hat

        # 7: sticks
        for i in self.ids:
            frame = self._compose(i, u_safe.get(i), states[i], refs.get(i))
            self.links[i].set_rc(frame)
            vz_override = None
            if self.mode[i] != Mode.FULL:          # firmware is flying the height
                tel = self.links[i].telemetry
                fresh = tel.vario_m_s is not None and t - tel.t_altitude < 0.3
                vz_override = tel.vario_m_s if (fresh and self.airborne[i]) else 0.0
            self.act[i].push(t, frame.roll, frame.pitch, frame.throttle, self.airborne[i], vz_override)
            self.last[i] = {"ref": refs[i].pos if i in refs else None, "est": states[i].pos,
                            "vel": states[i].vel, "bias": states[i].bias, "health": states[i].health.value,
                            "mode": self.mode[i].value, "yaw": states[i].yaw,
                            "u": u_safe.get(i), "phase": self.phase.value, "tau": self.tau,
                            "rate": self.clock.rate}

    def _references(self, t: float, states) -> Dict[int, Setpoint]:
        refs: Dict[int, Setpoint] = {}
        if self.plan is not None and self.phase != Phase.DONE:
            tp = t - self._t_phase0
            if self.phase == Phase.TRANSFER and tp >= self.plan.t_transfer:
                self.phase, self._t_phase0 = Phase.FORMATION, t
                self.clock.start(t)
                log.info("formation started (tau = 0)")
            elif self.phase == Phase.SETTLE and tp >= self.plan.t_settle:
                self.phase = Phase.DONE
                for i in self.ids:
                    if self.mode[i] == Mode.FULL:
                        self.hold_here(i, Mode.FULL)
            if self.phase == Phase.FORMATION:
                prev = self.last
                errs = [float(np.linalg.norm(prev[i]["ref"] - states[i].pos)) for i in self.ids
                        if i in prev and prev[i]["ref"] is not None and self.mode[i] == Mode.FULL]
                self.tau = self.clock.tick(t, max(errs) if errs else 0.0)
                if self.tau >= self.plan.formation_duration:
                    self.phase, self._t_phase0 = Phase.SETTLE, t
                    log.info("formation complete (clock rate min %.2f)", self.clock.min_rate_seen)
            if self.phase != Phase.DONE:
                tp, rate = t - self._t_phase0, (self.clock.rate if self.phase == Phase.FORMATION else 1.0)
                mrefs = self.plan.reference(self.phase, tp, self.tau)
                if self.phase in (Phase.TRANSFER, Phase.FORMATION):
                    lx = self.plan.reference(self.phase, tp + self.lead_xy, self.tau + self.lead_xy * rate)
                    lz = self.plan.reference(self.phase, tp + self.lead_z, self.tau + self.lead_z * rate)
                    for i, sp in mrefs.items():
                        sp.acc[:2] = lx[i].acc[:2]
                        sp.vel[2], sp.acc[2] = lz[i].vel[2], lz[i].acc[2]
                for i, sp in mrefs.items():
                    if self.mode[i] == Mode.FULL:
                        refs[i] = sp
        for i in self.ids:
            if i not in refs and self.target[i] is not None:
                refs[i] = Setpoint(i, t, self.target[i].copy())
        for i, sp in refs.items():                 # soft geofence on every reference
            if self.mode[i] == Mode.FULL:
                sp.pos = self.fence.clip(sp.pos)
        return refs

    def _compose(self, i: int, u: Optional[np.ndarray], st=None, ref=None) -> RCFrame:
        base = self.base_frame(i)
        m = self.mode[i]
        if m == Mode.OFF or u is None or st is None:
            if m == Mode.NEUTRAL:
                return replace(base, roll=1500, pitch=1500, yaw=1500,
                               throttle=1500 if self.airborne[i] else base.throttle)
            return replace(base)
        rc_roll, rc_pitch = self.mapper.tilt_sticks(float(u[0]), float(u[1]), st.yaw)
        yaw_ref = ref.yaw if ref is not None else 0.0
        rc_yaw = self.mapper.yaw_stick(yaw_ref, st.yaw) if self.loc.est[i].yaw_ok else 1500
        if m == Mode.FULL:
            vz = self.ctrl[i].vz_from_az(float(u[2]), float(st.vel[2]))
            thr, _ = self.mapper.throttle(i, vz)
        else:
            thr = base.throttle
        return replace(base, roll=rc_roll, pitch=rc_pitch, yaw=rc_yaw, throttle=thr)
