# Hardware integration

How this software talks to the Pluto drones, how it relates to the interfaces
Drona Aviation provided, and what must be verified on real hardware.

## 1. Use of the provided interfaces

The problem statement asks teams to use the Pluto ROS 2 package, MagisV2
firmware and the PlutoControl library as the primary hardware interfaces, and
to test and document any additions. The MoM (§11-12) allows modified or custom
implementations that stay compatible with the Pluto hardware.

| Provided interface | How we use it | Modified? | Evidence of compatibility |
|---|---|---|---|
| **MagisV2 firmware** | Flown as shipped. Our controller and simulator are built on its own flight modes (angle mode, baro alt-hold, take-off/land commands, RC-loss failsafe), read from its source (`serial_msp.cpp`, `altitudehold.cpp`, `failsafe.cpp`, `pid.cpp`, `config.cpp`). | No | The firmware emulator (`sim/plant/pluto_firmware_model.py`) reproduces these modes; Airlink session 1 measures them. |
| **PlutoControl 2.0.0** | (a) Reference for every packet and sequence: our MSP encoder is byte-identical (live test). (b) **Selectable fallback link**: `link.type: plutocontrol` flies the missions through Drona's own library - its writer thread sends the RC frames and commands; we only decode the replies. | No (used as-is) | `tests/unit/test_msp_protocol.py::test_live_plutocontrol_byte_for_byte` and `tests/unit/test_links.py::test_plutocontrol_fallback_link_flies_the_emulator`; Task 1 flown in simulation through both links: 2.9 cm RMS (our link) vs 2.8 cm (PlutoControl). |
| **Pluto ROS 2 package** | Reference for message layout (`PlutoMsg`, `PlutoPilot`) and MSP framing. Not in the control path. | No | Our packets match its `Protocol.createPacketMSP`. |

### Why our own driver is the default link

| Need | Provided library behaviour | Our `MspLink` |
|---|---|---|
| 4-10 drones in one process | ROS 2 package: IP hard-coded, MSP state in module globals (two drones overwrite each other) | One object per drone |
| Negative roll/pitch | ROS 2 package `read16` does not sign-extend (-1.0 deg arrives as ~6553) | Signed decoding (`struct`) |
| Telemetry at 50 Hz without blocking | PlutoControl `get_*()` sends a request, blocks on `recv`, prints, and races its own writer thread for the socket | Async reader, checksum-checked stream parser |
| Synchronised RC to all drones | One writer thread per drone (frames up to ~22 ms apart) | One loop writes every drone's frame in the same tick (~1 ms apart) |
| Latency | Nagle buffering on | `TCP_NODELAY`, one write per tick, measured round-trip time |
| Link drops | Connection lost for good | Automatic reconnect with back-off (the firmware lands the drone itself on RC loss) |

Sequences copied exactly from PlutoControl 2.0.0 (`swarm/manager.py`):

- **take-off**: disarm frame (throttle 1300, AUX4 1200), then throttle 1800 + AUX4 1500 + `MSP_SET_COMMAND 1`, then throttle 1500 (alt-hold).
- **land**: `MSP_SET_COMMAND 2`, sticks unchanged.
- **disarm**: throttle 1300, AUX4 1200.

Issue found in PlutoControl 2.0.0 (not used by us): `trim()` sends `MSP_ACC_TRIM` (240, a read request) instead of `MSP_SET_ACC_TRIM` (239).

## 2. Additional dependencies

| Package | Why | Where | Tested |
|---|---|---|---|
| numpy, pyyaml | maths, config | core (required) | sim + CI |
| opencv-contrib-python | ArUco detection, PnP, camera capture | localization (`vision` extra) | sim + CI (OpenCV 4.7+ API) |
| pyrealsense2 | D455f colour stream (final demo only) | `realsense` extra | to test on our ground station |
| plutocontrol 2.0.0 | fallback link + compatibility test | `plutocontrol` extra | CI (emulator) |
| v4l-utils (`v4l2-ctl`) | lock C920 focus/exposure/white balance | system package in Docker | to test at Airlink |

## 3. Wi-Fi setup

Each drone's ESP module: telnet to 192.168.4.1:23, then `+++AT MODE 3` (AP+STA:
keeps the drone's own access point as a fallback) and
`+++AT STA <ssid> <password>`; give each drone a fixed IP and put it in
`config/swarm.yaml`. A 2023 team found STA-only mode rejoined the router
unreliably after reboots.

## 4. Facts to verify at Airlink session 1 (in this order)

All plant numbers in `config/plant/pluto_v5.yaml` come from reading the firmware,
not from measurement. Each test is small, props-guards on, one drone.

1. **Stick signs** (`config/control.yaml` `rc_signs`): hover, then one 0.5 s
   pulse of +40 us on roll, pitch, yaw in turn. Expected: roll -> moves toward the
   drone's right; pitch -> moves forward; yaw -> turns counter-clockwise seen from
   above. Wrong sign -> flip it in the config. (The divergence watchdog lands a
   drone whose error grows past 0.4 m for 1 s, which catches a wrong sign.)
2. **Alt-hold response** (most important - see the robustness table in
   `testing_and_validation.md`): steps of +-60 us throttle -> fit
   `alt_hold_cm_s_per_us`, `alt_hold_deadband_us`, `z_tau_s`.
3. **Tilt response**: small roll/pitch steps -> fit `stick_to_tilt_deg_per_us`,
   `attitude_tau_s`, `drag_per_s`.
4. **Delay**: the logged `rtt_ms` column -> `model.command_delay_s` = RTT / 2.
5. Re-run `python -m pluto_swarm.control.design` and paste the gains into
   `config/control.yaml`.

## 5. Cameras

| | Airlink (Drona's) | Final demo (ours) |
|---|---|---|
| Camera | Logitech C920, `config/camera/c920.yaml` | RealSense D455f, colour only, `config/camera/d455_rgb.yaml` |
| Driver | `localization/cameras/opencv_cam.py` | `localization/cameras/realsense_cam.py` |
| Calibration | ask Drona for ~20 ChArUco images at fixed focus, >= 1 working day before session 1 | ChArUco ourselves |

The C920's short axis covers only about 1.8 m at hover height when mounted
3.5 m up. Take-off spots and formations must sit inside that view; a drone that
leaves it loses tracking and lands (by design).
