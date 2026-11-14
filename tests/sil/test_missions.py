"""Software-in-the-loop: the WHOLE stack (MSP over TCP, synthetic C920, PnP,
Kalman filter, control, safety) against the firmware emulator, in real time.
Shortened missions (1 ramp + 1 cycle) keep CI fast; the full-length runs are in
docs/testing_and_validation.md."""

import asyncio
import csv
import shutil

import numpy as np
import pytest
import yaml

from pluto_swarm.apps.run_mission import run

pytest.importorskip("cv2")
QUICK = {"cycles": 1, "ramp_periods": 1}
BUDGET_M = 0.10                     # MoM: max position error +-10 cm


@pytest.mark.parametrize("mission", ["task1_line_wave", "task2_circle_wave"])
def test_task_meets_error_budget_and_separation(mission):
    res = {}
    code = asyncio.run(run("sim", mission, overrides=QUICK, results=res))
    assert code == 0, res
    assert res["worst_max_err_m"] < BUDGET_M, res
    assert res["min_separation_m"] >= 0.514, res
    assert res["sync_spread_ms"] is not None and res["sync_spread_ms"] <= 50, res


def test_lost_marker_lands_that_drone_and_aborts_safely():
    res = {}
    code = asyncio.run(run("sim", "task2_circle_wave", overrides=QUICK, fault="hide:2:4", results=res))
    assert code == 7                                 # mission aborted on purpose
    assert "drone 2" in (res["abort_reason"] or "")
    with open(res["log"]) as f:
        rows = list(csv.DictReader(f))
    for d in ("1", "2", "3", "4"):
        last = [r for r in rows if r["drone"] == d][-1]
        assert float(last["true_z"]) < 0.05          # everyone is on the ground


def test_ten_drones_scale(tmp_path, monkeypatch):
    """N is config: 10 drones fly the circle task (wide camera so all fit in view)."""
    shutil.copytree(pytest.importorskip("pluto_swarm.core.config").repo_root() / "config", tmp_path / "config")
    sw = {"drones": [{"id": i + 1, "ip": "0.0.0.0", "port": 23, "marker_id": i + 1,
                      "start_slot": [0.4 + 0.55 * (i % 5), 0.9 + 1.2 * (i // 5)]} for i in range(10)]}
    (tmp_path / "config/swarm.yaml").write_text(yaml.safe_dump(sw))
    prof = yaml.safe_load((tmp_path / "config/profiles/sim.yaml").read_text())
    prof["camera"]["model"] = "config/camera/d455_rgb.yaml"
    (tmp_path / "config/profiles/sim.yaml").write_text(yaml.safe_dump(prof))
    monkeypatch.setenv("PLUTO_SWARM_ROOT", str(tmp_path))
    res = {}
    code = asyncio.run(run("sim", "task2_circle_wave",
                           overrides={**QUICK, "radius_m": 1.0, "centre": [1.5, 1.5]}, results=res))
    assert code == 0, res
    assert len(res["drones"]) == 10
    assert res["min_separation_m"] >= 0.514
    assert res["worst_max_err_m"] < BUDGET_M
