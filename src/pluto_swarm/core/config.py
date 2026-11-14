"""Load the YAML config files and apply a profile.

A profile (config/profiles/*.yaml) only chooses backends - which link and which
camera. Everything else (drones, arena, plant, control, safety, mission) is shared.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from .types import DroneConfig


def repo_root() -> Path:
    """Repo root = folder that contains config/. Override with PLUTO_SWARM_ROOT."""
    env = os.environ.get("PLUTO_SWARM_ROOT")
    if env:
        return Path(env).resolve()
    here = Path(__file__).resolve()
    for parent in [Path.cwd(), *here.parents]:
        if (parent / "config" / "swarm.yaml").exists():
            return parent
    raise FileNotFoundError("Cannot find config/swarm.yaml - run from the repo folder "
                            "or set PLUTO_SWARM_ROOT.")


def load_yaml(path: Path) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


@dataclass
class Config:
    profile: Dict[str, Any]
    drones: List[DroneConfig]
    arena: Dict[str, Any]
    plant: Dict[str, Any]
    control: Dict[str, Any]
    safety: Dict[str, Any]
    mission: Dict[str, Any]
    root: Path


def load_config(profile: str = "sim", mission: str = "hover_test",
                root: Optional[Path] = None) -> Config:
    root = Path(root) if root else repo_root()
    c = root / "config"
    prof = load_yaml(c / "profiles" / f"{profile}.yaml")
    swarm = load_yaml(c / "swarm.yaml")

    link = prof.get("link", {})
    host_override = link.get("host_override")
    base_port = link.get("base_port")

    drones: List[DroneConfig] = []
    seen = set()
    for i, d in enumerate(swarm["drones"]):
        if d["id"] in seen:
            raise ValueError(f"Duplicate drone id {d['id']} in swarm.yaml")
        seen.add(d["id"])
        drones.append(DroneConfig(
            id=int(d["id"]),
            host=host_override or d["ip"],
            port=int(base_port + i) if base_port else int(d["port"]),
            marker_id=int(d.get("marker_id", d["id"])),
            start_slot=(float(d["start_slot"][0]), float(d["start_slot"][1])),
        ))

    return Config(
        profile=prof,
        drones=drones,
        arena=load_yaml(c / "arena.yaml"),
        plant=load_yaml(c / "plant" / "pluto_v5.yaml"),
        control=load_yaml(c / "control.yaml"),
        safety=load_yaml(c / "safety.yaml"),
        mission=load_yaml(c / "missions" / f"{mission}.yaml"),
        root=root,
    )
