"""SimWorld - N emulated Pluto drones, each behind its own MSP/TCP server.

Two ways to use it:
  * in-process (default): apps/run_mission.py starts it when the profile says
    sim.spawn: true, so ONE terminal command runs sim + swarm software;
  * standalone:  python -m sim.plant.world --profile sim
    (then run the swarm software in a second terminal, or Gazebo's pose_bridge).

world.truth() gives exact positions - for logging and for the synthetic camera
ONLY. Never feed truth into the controller when testing localization.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
from typing import Dict, List

import numpy as np

from pluto_swarm.core.clock import now
from pluto_swarm.core.config import Config, load_config

from .msp_server import SimDroneServer
from .pluto_firmware_model import PlutoFirmwareModel

log = logging.getLogger(__name__)


class SimWorld:
    def __init__(self, cfg: Config) -> None:
        sim = cfg.profile.get("sim", {})
        self.physics_hz = float(sim.get("physics_hz", 200))
        seed = sim.get("seed")
        root_rng = np.random.default_rng(seed)
        # Robustness testing: the simulated drones may differ from the plant the
        # controller assumes (sim.plant_overrides), e.g. a slower attitude loop.
        plant = {**cfg.plant, **(sim.get("plant_overrides") or {})}
        self.plant = plant
        self.servers: List[SimDroneServer] = []
        for d in cfg.drones:
            rng = np.random.default_rng(root_rng.integers(1 << 31))
            model = PlutoFirmwareModel(plant, start_xy=d.start_slot, rng=rng)
            self.servers.append(SimDroneServer(
                d.id, model, d.host, d.port,
                delay_s=float(sim.get("link_delay_s", 0.0)),
                jitter_s=float(sim.get("link_jitter_s", 0.0)), rng=rng))
        self._task = None
        self._running = False

    async def start(self) -> None:
        for s in self.servers:
            await s.start()
        self._running = True
        self._task = asyncio.ensure_future(self._physics_loop())
        log.info("sim world: %d drones on ports %s", len(self.servers),
                 [s.port for s in self.servers])

    async def stop(self) -> None:
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        for s in self.servers:
            await s.stop()

    async def _physics_loop(self) -> None:
        dt_nom = 1.0 / self.physics_hz
        t_last = now()
        while self._running:
            await asyncio.sleep(dt_nom)
            t = now()
            dt = min(t - t_last, 0.05)              # cap: never jump more than 50 ms
            t_last = t
            for s in self.servers:
                s.process_due()
                # sub-step so physics stays stable even if the loop was late
                n = max(1, int(round(dt / dt_nom)))
                for _ in range(n):
                    s.model.step(dt / n)

    def truth(self) -> Dict[int, dict]:
        out = {}
        for s in self.servers:
            m = s.model
            out[s.drone_id] = {"pos": m.s.pos.copy(), "vel": m.s.vel.copy(),
                               "yaw": m.s.yaw, "phi": m.s.phi, "theta": m.s.theta,
                               "phase": m.phase.value}
        return out


async def _run_standalone(profile: str) -> None:
    cfg = load_config(profile)
    world = SimWorld(cfg)
    await world.start()
    print(f"Sim running: {len(world.servers)} drones on 127.0.0.1 ports "
          f"{[s.port for s in world.servers]}. Ctrl+C to stop.")
    try:
        while True:
            await asyncio.sleep(1.0)
            line = "  ".join(f"#{i}:{v['phase'][:4]} z={v['pos'][2]:.2f}"
                             for i, v in world.truth().items())
            print(line, flush=True)
    finally:
        await world.stop()


def main() -> None:
    ap = argparse.ArgumentParser(description="Run the Pluto firmware emulator swarm")
    ap.add_argument("--profile", default="sim")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    try:
        asyncio.run(_run_standalone(args.profile))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
