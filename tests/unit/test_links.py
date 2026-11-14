"""Drone links: auto-reconnect (MspLink) and the PlutoControl fallback."""

import asyncio

import numpy as np
import pytest

from pluto_swarm.core.config import load_config
from pluto_swarm.core.types import DroneConfig, RCFrame
from pluto_swarm.hal.msp_link import MspLink
from sim.plant.msp_server import SimDroneServer
from sim.plant.pluto_firmware_model import PlutoFirmwareModel

CFG = load_config("sim", "hover_test")


async def _serve(port):
    model = PlutoFirmwareModel(CFG.plant, rng=np.random.default_rng(0))
    srv = SimDroneServer(1, model, "127.0.0.1", port)
    await srv.start()
    return srv


async def _pump(srv, seconds):
    """Deliver packets AND advance the emulator's physics (SimWorld does both)."""
    for _ in range(int(seconds / 0.01)):
        srv.process_due()
        srv.model.step(0.01)
        await asyncio.sleep(0.01)


def test_msp_link_measures_rtt_and_reconnects():
    async def main():
        port = 5899
        srv = await _serve(port)
        link = MspLink(DroneConfig(1, "127.0.0.1", port, 1, (0.0, 0.0)),
                       {**CFG.control, "link": {"auto_reconnect": True, "reconnect_backoff_max_s": 0.2}})
        await link.connect(2.0, own_write_loop=True)
        await _pump(srv, 0.5)
        assert link.rtt_s is not None and link.rtt_s < 0.1
        await srv.stop()                            # drone "reboots"
        await asyncio.sleep(0.3)
        srv = await _serve(port)
        await _pump(srv, 1.5)
        assert link.connected and link.reconnects >= 1
        await link.close()
        await srv.stop()
    asyncio.run(main())


def test_plutocontrol_fallback_link_flies_the_emulator():
    pytest.importorskip("plutocontrol")
    from pluto_swarm.hal.plutocontrol_link import PlutoControlLink

    async def main():
        port = 5898
        srv = await _serve(port)
        link = PlutoControlLink(DroneConfig(1, "127.0.0.1", port, 1, (0.0, 0.0)), CFG.control)
        await link.connect(3.0)
        link.set_rc(RCFrame(throttle=1800, aux4=1500))
        from pluto_swarm.core.types import FirmwareCommand
        link.send_command(FirmwareCommand.TAKE_OFF)
        await _pump(srv, 2.5)
        assert srv.model.s.pos[2] > 0.5             # their writer thread + our decoder
        assert link.telemetry.alt_m is not None and link.telemetry.alt_m > 0.4
        await link.close()
        await srv.stop()
    asyncio.run(main())
