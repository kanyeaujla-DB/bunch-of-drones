# Synchronized Multi-Drone Swarm Execution
**Inter IIT Tech Meet 15.0 — Drona Aviation problem statement**

Ground-station software that flies a swarm of Pluto drones in synchronized line
and circle wave formations, using an overhead camera for localization.

## Architecture in one picture

```
            ┌──────────────── one Python process ────────────────┐
 camera ──► │ localization → planning → control → safety → swarm │ ──► DroneLink ──► MSP/TCP ──► drone
 (swappable)│        all driven by ONE shared mission clock      │     (swappable)
            └────────────────────────────────────────────────────┘
```

The simulator imitates the Pluto **firmware** and speaks the same MSP-over-TCP
protocol as a real drone, so going from simulation to real flight is a
**config-profile change, not a code change**:

| Profile | Drones | Camera |
|---|---|---|
| `sim` | firmware emulator (127.0.0.1) | synthetic (P1) |
| `gazebo` | firmware emulator + Gazebo rendering | Gazebo camera (P1) |
| `airlink` | real Plutos at the test site | Drona's Logitech C920 |
| `final_demo` | real Plutos | our RealSense D455f, colour stream only |

## Quick start (Ubuntu 22.04)

```bash
git clone <repo-url> pluto_swarm && cd pluto_swarm
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
pytest -q                                         # all tests should pass
pluto-run --profile sim --mission hover_test      # 4 simulated drones: take off, hover, land
```

Flight logs are written to `logs/`. Ctrl+C once = land all; twice = disarm all.

## Repository map

| Path | What lives there |
|---|---|
| `config/` | every number: drones, arena, plant model, gains, safety, missions, profiles |
| `src/pluto_swarm/core/` | shared types (the contract), config loader, mission clock |
| `src/pluto_swarm/hal/` | MSP protocol + async TCP link to one drone |
| `src/pluto_swarm/localization/` | camera → marker → pose → filter → health |
| `src/pluto_swarm/planning/` | formations, waves, smooth transitions |
| `src/pluto_swarm/control/` | position / height controllers, RC mapping |
| `src/pluto_swarm/swarm/` | flight state machine, synchronized tick loop |
| `src/pluto_swarm/safety/` | preflight, separation, geofence, watchdogs |
| `src/pluto_swarm/monitoring/` | logging, metrics, visualization |
| `src/pluto_swarm/apps/` | command-line entry points |
| `sim/plant/` | Pluto firmware emulator + MSP server |
| `sim/gazebo/` | Gazebo world, models, pose bridge (rendering + camera only) |
| `tests/` | unit tests and software-in-the-loop missions |
| `docs/` | technical documentation |

See `docs/setup_guide.md` for the full setup and team Git workflow.

## Status
- [x] P0 — MSP driver (byte-identical to the official client), firmware emulator, N-drone take-off/hover/land
- [ ] P1 — one drone, closed-loop position hold
- [ ] P2 — two drones + safety layers
- [ ] P3 — four drones, both tasks
- [ ] P4 — delivery
