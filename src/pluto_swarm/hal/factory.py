"""Pick the DroneLink implementation from the profile (`link.type`)."""

from __future__ import annotations

from typing import Dict

from ..core.types import DroneConfig
from .base import DroneLink


def make_link(drone: DroneConfig, profile: Dict, control_cfg: Dict) -> DroneLink:
    kind = (profile.get("link") or {}).get("type", "msp")
    if kind == "msp":
        from .msp_link import MspLink
        return MspLink(drone, control_cfg)
    if kind == "plutocontrol":
        from .plutocontrol_link import PlutoControlLink
        return PlutoControlLink(drone, control_cfg)
    raise ValueError(f"unknown link.type {kind!r} (expected 'msp' or 'plutocontrol')")
