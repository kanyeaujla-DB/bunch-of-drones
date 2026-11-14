"""Software-in-the-loop: the full stack over real TCP sockets against the emulator.
This is the P0 exit test: N drones take off, hover and land from one command."""

import asyncio
import csv

from pluto_swarm.apps.run_mission import run


def test_hover_mission_all_drones_land(tmp_path, monkeypatch):
    code = asyncio.run(run("sim", "hover_test", hover_s=1.0))
    assert code == 0


def test_hover_log_has_every_drone_flying():
    import glob
    import os
    from pluto_swarm.core.config import repo_root
    logs = sorted(glob.glob(str(repo_root() / "logs" / "*_sim_hover_test.csv")), key=os.path.getmtime)
    assert logs, "run the mission test first"
    with open(logs[-1]) as f:
        rows = list(csv.DictReader(f))
    flying = {r["drone"] for r in rows if r["phase"] == "flying"}
    assert flying == {"1", "2", "3", "4"}
    peak = max(float(r["true_z"]) for r in rows if r["true_z"])
    assert 1.0 < peak < 1.4
