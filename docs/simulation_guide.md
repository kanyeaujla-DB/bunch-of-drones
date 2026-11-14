# Simulation guide

The simulator imitates the Pluto **firmware** and speaks MSP over TCP, so the
swarm software cannot tell it from real drones. One command runs everything:

```bash
pluto-run --profile sim --mission task1_line_wave      # line + travelling height wave
pluto-run --profile sim --mission task2_circle_wave    # rotating circle + in-phase Z
pluto-run --profile sim --mission hover_test
```

Useful options:

| Option | Example | Purpose |
|---|---|---|
| `--set` | `--set cycles=1 --set ramp_periods=1` | shorter mission |
| `--camera` | `--camera truth` | perfect positions (isolates control from localization) |
| `--fault` | `--fault hide:2:5` | hide drone 2's marker 5 s into the mission |
| `--sim-plant` | `--sim-plant z_tau_s=0.5` | simulated drone differs from the model (robustness) |

Each run writes `logs/<time>_<profile>_<mission>.csv` and prints a METRICS line
(per-drone RMS/max error, min separation, synchronisation).
`python -m pluto_swarm.monitoring.metrics <log>` recomputes them.

## What is simulated

| Part | Model |
|---|---|
| Firmware | angle mode (stick -> tilt, 20 deg clamp, attitude lag), baro alt-hold (climb-rate request, +-40 us deadband), take-off/land commands, land on RC loss |
| Airframe | 54 g Pluto 1.2 (SDF), linear drag, per-drone trim error |
| Sensors | baro noise + drift, attitude telemetry, battery sag |
| Wi-Fi | delay + jitter on every packet |
| Camera | Logitech C920 (1080p30, rolling shutter, 0.3 px noise, 3 % dropouts, 35 ms latency) projecting the real 8 cm ArUco corners; poses solved with the same PnP code as on hardware |

Gazebo (`sim/gazebo/`) renders the same emulator poses for demo videos; it is
not the physics.
