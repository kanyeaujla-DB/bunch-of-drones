"""Every profile and mission must load, and every file a profile points to must exist."""

from pathlib import Path

import pytest
import yaml

from pluto_swarm.core.config import load_config, repo_root

ROOT = repo_root()
PROFILES = sorted(p.stem for p in (ROOT / "config" / "profiles").glob("*.yaml"))
MISSIONS = sorted(p.stem for p in (ROOT / "config" / "missions").glob("*.yaml"))


@pytest.mark.parametrize("profile", PROFILES)
def test_profile_loads_and_its_files_exist(profile):
    cfg = load_config(profile=profile, mission="hover_test", root=ROOT)
    cam = cfg.profile.get("camera", {})
    for key in ("model", "extrinsics"):
        if cam.get(key):
            path = ROOT / cam[key]
            assert path.exists(), f"{profile}: camera.{key} -> missing {path}"
            yaml.safe_load(path.read_text())


@pytest.mark.parametrize("mission", MISSIONS)
def test_mission_loads(mission):
    cfg = load_config(profile="sim", mission=mission, root=ROOT)
    assert cfg.mission["name"] == mission


def test_camera_models_keep_depth_off_and_automatics_locked():
    c920 = yaml.safe_load((ROOT / "config/camera/c920.yaml").read_text())
    d455 = yaml.safe_load((ROOT / "config/camera/d455_rgb.yaml").read_text())
    assert d455["enable_depth"] is False          # same algorithm on both cameras
    ctl = c920["v4l2_controls"]
    assert ctl["focus_automatic_continuous"] == 0
    assert ctl["auto_exposure"] == 1              # manual
    assert d455["rs_options"]["enable_auto_exposure"] == 0


def test_task1_is_a_vertical_wave_along_a_fixed_line():
    m = yaml.safe_load((ROOT / "config/missions/task1_line_wave.yaml").read_text())
    assert m["wave_axis"] == "z" and m["line_axis"] == "x"
    assert m["direction"] in (1, -1)


def test_formation_spacing_respects_minimum_separation():
    safe_min = 0.514  # 16.4 cm airframe + 15 cm gap + 2 x 10 cm position error
    t1 = yaml.safe_load((ROOT / "config/missions/task1_line_wave.yaml").read_text())
    t2 = yaml.safe_load((ROOT / "config/missions/task2_circle_wave.yaml").read_text())
    import math
    n = len(yaml.safe_load((ROOT / "config/swarm.yaml").read_text())["drones"])
    assert t1["spacing_m"] >= safe_min
    assert 2 * t2["radius_m"] * math.sin(math.pi / n) >= safe_min
