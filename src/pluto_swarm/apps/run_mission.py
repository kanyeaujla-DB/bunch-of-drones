"""Run a mission on a profile - THE one command for sim, Airlink and final demo.

    pluto-run --profile sim --mission task1_line_wave      # emulator + synthetic camera
    pluto-run --profile sim --mission task2_circle_wave
    pluto-run --profile airlink --mission hover_test        # real drones, Drona's C920
    pluto-run --profile sim --mission task1_line_wave --set cycles=1 --set ramp_periods=1

Camera source comes from the profile (`camera.type`):
    none       open loop (P0): take off, hover on the firmware's own hold, land
    synthetic  simulator truth -> pixels -> PnP -> filter (the full pipeline)
    truth      simulator truth straight into the filter (debug control only)
    opencv     USB camera (C920)          realsense  RealSense colour stream

Ctrl+C once = land all.  Ctrl+C twice = disarm all (emergency).
Fault injection (sim): --fault hide:2:5  hides drone 2's marker 5 s into the mission.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import signal
import sys
from typing import Dict, List, Optional

import yaml

from ..core.config import load_config, load_yaml
from ..monitoring.data_logger import DataLogger
from ..swarm.manager import SwarmManager

log = logging.getLogger("run_mission")


def _parse_overrides(items: Optional[List[str]]) -> Dict:
    out = {}
    for it in items or []:
        k, _, v = it.partition("=")
        out[k.strip()] = yaml.safe_load(v)
    return out


def _build_localization(cfg, world, kind: str):
    """Returns (localizer, runner, synthetic_camera_or_None)."""
    from ..localization.localizer import (Localizer, SyntheticRunner, ThreadedCameraRunner,
                                          TruthRunner)
    loc_cfg = load_yaml(cfg.root / "config" / "localization.yaml")
    markers = {d.id: d.marker_id for d in cfg.drones}
    tau_z = float(cfg.plant["z_tau_s"])
    drag = float(cfg.plant.get("drag_per_s", 0.0))
    cam_cfg = cfg.profile.get("camera", {})
    if kind == "truth":
        if world is None:
            raise RuntimeError("camera.type 'truth' only exists in simulation")
        loc = Localizer(markers, loc_cfg, cfg.safety, tau_z, drag=drag)
        return loc, TruthRunner(world.truth, loc, fps=30.0, noise_m=0.003, latency_s=0.03), None
    from ..localization.camera_model import load_camera_model
    from ..localization.pose import PoseSolver
    model = load_camera_model(cfg.root, cam_cfg)
    loc = Localizer(markers, loc_cfg, cfg.safety, tau_z, PoseSolver(model, loc_cfg), drag=drag)
    if kind == "synthetic":
        if world is None:
            raise RuntimeError("camera.type 'synthetic' needs the simulator (sim.spawn: true)")
        from ..localization.cameras.synthetic_cam import SyntheticCamera
        cam = SyntheticCamera(model, world.truth, markers, loc_cfg, cam_cfg.get("synthetic", {}),
                              rng=None)
        return loc, SyntheticRunner(cam, loc, model.fps), cam
    from ..localization.detectors.aruco import ArucoDetector
    cam_file = yaml.safe_load((cfg.root / cam_cfg["model"]).read_text())
    if kind == "opencv":
        from ..localization.cameras.opencv_cam import OpenCVCamera
        camera = OpenCVCamera(cam_file, cam_cfg.get("device", 0), float(cam_cfg.get("usb_latency_s", 0.0)))
    elif kind == "realsense":
        from ..localization.cameras.realsense_cam import RealSenseCamera
        camera = RealSenseCamera(cam_file, cam_cfg.get("serial"))
    else:
        raise ValueError(f"unknown camera.type {kind!r}")
    return loc, ThreadedCameraRunner(camera, ArucoDetector(loc_cfg), loc), None


async def run(profile: str, mission: str, hover_s: Optional[float] = None,
              log_rate_hz: float = 25.0, overrides: Optional[Dict] = None,
              camera: Optional[str] = None, fault: Optional[str] = None,
              results: Optional[Dict] = None, sim_plant: Optional[Dict] = None,
              sim_overrides: Optional[Dict] = None) -> int:
    cfg = load_config(profile, mission)
    cfg.mission.update(overrides or {})
    if sim_plant or sim_overrides:
        sim = cfg.profile.setdefault("sim", {})
        sim.update(sim_overrides or {})
        sim["plant_overrides"] = {**(sim.get("plant_overrides") or {}), **(sim_plant or {})}
    if hover_s is not None:
        cfg.mission["hover_s"] = hover_s
    world = None
    if cfg.profile.get("sim", {}).get("spawn", False):
        from sim.plant.world import SimWorld          # only imported for sim profiles
        world = SimWorld(cfg)
        await world.start()

    mgr = SwarmManager(cfg)
    kind = camera or cfg.profile.get("camera", {}).get("type", "none")
    runner, syn_cam = None, None
    if kind != "none":
        loc, runner, syn_cam = _build_localization(cfg, world, kind)
        mgr.attach_flight(loc)
    logger = DataLogger(cfg.root / "logs", f"{cfg.profile['name']}_{cfg.mission['name']}")

    # ---- operator kill switch: Ctrl+C
    presses = {"n": 0}
    loop = asyncio.get_running_loop()

    def on_sigint():
        presses["n"] += 1
        if presses["n"] == 1:
            log.warning("Ctrl+C -> LANDING all (press again to DISARM)")
            mgr.abort_requested = True
            if mgr.flight is not None:
                mgr.flight.stop_mission("operator Ctrl+C")
        else:
            mgr.disarm_all()
    try:
        loop.add_signal_handler(signal.SIGINT, on_sigint)
    except (NotImplementedError, RuntimeError):          # Windows / non-main thread
        pass

    async def log_loop():
        while True:
            truth = world.truth() if world else {}
            fl = mgr.flight
            for i, l in mgr.links.items():
                snap = fl.last.get(i) if fl is not None else None
                logger.log(i, mgr.phase[i].value, l, truth.get(i), snap,
                           fl.ticks if fl is not None else None)
            await asyncio.sleep(1.0 / log_rate_hz)

    async def inject_fault():
        kind_, did, after = fault.split(":")
        while mgr.flight is None or mgr.flight.plan is None:
            await asyncio.sleep(0.1)
        await asyncio.sleep(float(after))
        if kind_ == "hide" and syn_cam is not None:
            log.warning("FAULT INJECTED: drone %s marker hidden", did)
            syn_cam.blind.add(int(did))

    exit_code = 0
    tasks = []
    try:
        problems = await mgr.connect_all()
        if problems:
            for p in problems:
                log.error(p)
            return 2
        if runner is not None:
            runner.start()
        tasks.append(asyncio.ensure_future(log_loop()))
        if fault:
            tasks.append(asyncio.ensure_future(inject_fault()))

        problems = await mgr.preflight()
        if mgr.flight is not None and not problems:
            problems = await mgr.wait_tracking()
        if problems:
            for p in problems:
                log.error("PREFLIGHT FAIL: %s", p)
            return 3
        log.info("preflight OK for drones %s", list(mgr.links))

        if mgr.flight is None:
            if cfg.mission.get("type") not in ("hover", None):
                log.error("mission %r needs a camera (profile camera.type is 'none')", mission)
                return 3
            ok = await mgr.takeoff_all()
            if not ok:
                log.error("take-off did not reach height in time (or aborted)")
                exit_code = 4
            if ok and not mgr.abort_requested:
                await mgr.hover(float(cfg.mission.get("hover_s", 8)))
        else:
            limits = cfg.safety.get("limits", {})
            exit_code = await mgr.fly_mission(cfg.mission, {
                "v_max": limits.get("transfer_v_max_m_s", 0.4),
                "a_max": limits.get("transfer_a_max_m_s2", 0.8),
                "t_min": limits.get("transfer_t_min_s", 2.0)})
        landed = await mgr.land_all()
        if not landed:
            log.error("not all drones confirmed landed - they were disarmed anyway")
            exit_code = exit_code or 5
        await asyncio.sleep(0.5)
    finally:
        for t in tasks:
            t.cancel()
        if runner is not None:
            await runner.stop()
        logger.close()
        await mgr.close_all()
        if world:
            for s in world.servers:
                for e in s.model.events:
                    log.info("sim drone %d: %s", s.drone_id, e)
            await world.stop()
        log.info("log written: %s (%d rows); late ticks %d / %d",
                 logger.path, logger.rows, mgr.ticker.late_ticks, mgr.ticker.ticks)
        rtts = {i: round(1000 * l.rtt_s, 1) for i, l in mgr.links.items() if getattr(l, "rtt_s", None)}
        if rtts:
            log.info("link round-trip ms: %s", rtts)
        if mgr.flight is not None:
            for e in mgr.flight.watch.events:
                log.warning("watchdog: %s", e)
            log.info("safety filter interventions: %d", mgr.flight.cbf.interventions)
            if mgr.flight.plan is not None:
                from ..monitoring.metrics import compute
                m = compute(str(logger.path))
                log.info("METRICS %s", json.dumps(m, default=str))
                if results is not None:
                    results.update(m)
        if results is not None:
            results["log"] = str(logger.path)
            results["exit_code"] = exit_code
            results["abort_reason"] = mgr.flight.abort_reason if mgr.flight else None
            results["events"] = list(mgr.flight.watch.events) if mgr.flight else []
            results["clock_rate_min"] = mgr.flight.clock.min_rate_seen if mgr.flight else None
    return exit_code


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Run a Pluto swarm mission")
    ap.add_argument("--profile", default="sim", help="config/profiles/<name>.yaml")
    ap.add_argument("--mission", default="hover_test", help="config/missions/<name>.yaml")
    ap.add_argument("--hover", type=float, default=None, help="override hover seconds")
    ap.add_argument("--set", action="append", metavar="KEY=VALUE",
                    help="override a mission parameter (repeatable)")
    ap.add_argument("--camera", default=None, help="override camera.type (e.g. truth, none)")
    ap.add_argument("--fault", default=None, help="sim fault injection, e.g. hide:2:5")
    ap.add_argument("--sim-plant", action="append", metavar="KEY=VALUE",
                    help="make the SIMULATED drone differ from the model (robustness test)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    sys.exit(asyncio.run(run(args.profile, args.mission, args.hover,
                             overrides=_parse_overrides(args.set), camera=args.camera,
                             fault=args.fault, sim_plant=_parse_overrides(args.sim_plant))))


if __name__ == "__main__":
    main()
