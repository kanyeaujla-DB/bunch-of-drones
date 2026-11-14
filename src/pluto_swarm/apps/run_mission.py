"""Run a mission on a profile - THE one command for sim, Airlink and final demo.

    pluto-run --profile sim --mission hover_test         # emulator, one terminal
    pluto-run --profile airlink --mission hover_test     # real drones (IPs from swarm.yaml)

Ctrl+C once = land all.  Ctrl+C twice = disarm all (emergency).
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from typing import Optional

from ..core.config import load_config
from ..monitoring.data_logger import DataLogger
from ..swarm.manager import SwarmManager

log = logging.getLogger("run_mission")


async def run(profile: str, mission: str, hover_s: Optional[float] = None,
              log_rate_hz: float = 10.0) -> int:
    cfg = load_config(profile, mission)
    world = None
    if cfg.profile.get("sim", {}).get("spawn", False):
        from sim.plant.world import SimWorld          # only imported for sim profiles
        world = SimWorld(cfg)
        await world.start()

    mgr = SwarmManager(cfg)
    logger = DataLogger(cfg.root / "logs", f"{cfg.profile['name']}_{cfg.mission['name']}")

    # ---- operator kill switch: Ctrl+C
    presses = {"n": 0}
    loop = asyncio.get_running_loop()

    def on_sigint():
        presses["n"] += 1
        if presses["n"] == 1:
            log.warning("Ctrl+C -> LANDING all (press again to DISARM)")
            mgr.abort_requested = True
        else:
            mgr.disarm_all()
    try:
        loop.add_signal_handler(signal.SIGINT, on_sigint)
    except NotImplementedError:          # Windows
        pass

    async def log_loop():
        while True:
            truth = world.truth() if world else {}
            for i, l in mgr.links.items():
                logger.log(i, mgr.phase[i].value, l, truth.get(i))
            await asyncio.sleep(1.0 / log_rate_hz)

    exit_code = 0
    log_task = None
    try:
        problems = await mgr.connect_all()
        if problems:
            for p in problems:
                log.error(p)
            return 2
        log_task = asyncio.ensure_future(log_loop())

        problems = await mgr.preflight()
        if problems:
            for p in problems:
                log.error("PREFLIGHT FAIL: %s", p)
            return 3
        log.info("preflight OK for drones %s", list(mgr.links))

        ok = await mgr.takeoff_all()
        if not ok:
            log.error("take-off did not reach height in time (or aborted)")
            exit_code = 4
        if ok and not mgr.abort_requested:
            if cfg.mission.get("type") != "hover":
                log.error("mission type %r not implemented yet (P3) - hovering instead",
                          cfg.mission.get("type"))
            await mgr.hover(hover_s if hover_s is not None else float(cfg.mission.get("hover_s", 8)))
        landed = await mgr.land_all()
        if not landed:
            log.error("not all drones confirmed landed - they were disarmed anyway")
            exit_code = exit_code or 5
        await asyncio.sleep(0.5)
    finally:
        if log_task:
            log_task.cancel()
        logger.close()
        await mgr.close_all()
        if world:
            for s in world.servers:
                for e in s.model.events:
                    log.info("sim drone %d: %s", s.drone_id, e)
            await world.stop()
        log.info("log written: %s (%d rows); late ticks %d / %d",
                 logger.path, logger.rows, mgr.ticker.late_ticks, mgr.ticker.ticks)
    return exit_code


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Run a Pluto swarm mission")
    ap.add_argument("--profile", default="sim", help="config/profiles/<name>.yaml")
    ap.add_argument("--mission", default="hover_test", help="config/missions/<name>.yaml")
    ap.add_argument("--hover", type=float, default=None, help="override hover seconds")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    sys.exit(asyncio.run(run(args.profile, args.mission, args.hover)))


if __name__ == "__main__":
    main()
