# Project rules for Claude Code (read by every teammate's Claude automatically)

## What this is
Ground-station software for a synchronized swarm of Pluto (Primus V5) drones —
Inter IIT Tech Meet 15.0, Drona Aviation. Deadline 5 Dec 2026.
Governing design: "Pluto Swarm — Workflow Analysis" (team architecture doc).

## Hard rules
1. **Anonymity.** Never write any person's name, roll number, college, email or
   location into code, comments, docs, commit messages or file names. The
   competition forbids it. Team = "Team Drona".
2. **One Python process, two swappable edges.** The core never imports a camera
   SDK, socket or simulator directly. Drones are reached only through
   `hal/base.py: DroneLink`; cameras only through `localization/cameras/base.py`.
3. **`src/pluto_swarm/core/types.py` is the contract.** Do not change a field
   without the whole team agreeing. Units: metres, seconds, radians. Arena
   frame: origin at a floor corner, Z up. Body frame: x forward, y left.
4. **No hard-coded numbers.** Drone count, IPs, gains, limits, plant constants
   live in `config/`. N is never assumed to be 4.
5. **`sim/` never imports hardware-only code** (only the shared MSP protocol).
6. **Plant facts are unverified** until Airlink session 1 — see
   `config/plant/pluto_v5.yaml`. Don't tune against guessed numbers blindly.
7. Target **Python 3.10** (Ubuntu 22.04): no `asyncio.TaskGroup`, no `StrEnum`.
8. Run `pytest -q` before every commit; add a test for every new pure function.

## Folder ownership — stay in your lane
| Folder | Owner |
|---|---|
| `src/pluto_swarm/control/`, `sim/plant/`, `config/plant/`, `config/control.yaml` | control + sim |
| `sim/gazebo/` (rendering, overhead camera, pose_bridge) | gazebo |
| `src/pluto_swarm/localization/`, `config/camera/` | localization |
| `src/pluto_swarm/planning/`, `swarm/`, `safety/` | swarm + safety |
| `docker/`, `tools/`, `monitoring/`, CI, `scripts/` | infra |
| `core/`, `hal/` | whole team (changes need review) |

Touching another lane? Open a PR and ask that owner to review.

## Git
- Never commit to `main` directly: branch `<area>/<task>` (e.g. `control/xy-loop`),
  push, open a pull request.
- Never commit `logs/*.csv`, build output, or secrets.

## Commands
```bash
pip install -e ".[dev]"                          # once (inside the venv)
pytest -q                                        # all tests
pluto-run --profile sim --mission hover_test     # 4 emulated drones, one terminal
python -m sim.plant.world --profile sim          # emulator alone (for Gazebo bridge)
```

## Phase plan (exit test must pass before moving on)
P0 foundation (done: MSP driver, emulator, hover mission) → P1 one drone closed
loop → P2 two drones + safety → P3 four drones, both tasks → P4 delivery (2 Dec freeze).
