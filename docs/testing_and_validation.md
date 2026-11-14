# Testing and validation

All results below are **SIMULATION** results (firmware emulator + synthetic
Logitech C920 + the full localization pipeline). Physical results from Airlink
will be added in a separate section and never mixed with these.

## 1. Test layers

| Layer | Command | What it proves |
|---|---|---|
| Unit | `pytest -q tests/unit` (~7 s) | Protocol bytes = PlutoControl; formations are exact derivatives; Kalman filter gives the same answer with late/out-of-order data; drift is learned; RC mapping inverts the firmware model; dither averages correctly; collision filter keeps two drones apart; plan validation; watchdog verdicts; link reconnect; PlutoControl fallback |
| Software in the loop | `pytest -q tests/sil` (~90 s) | Both tasks meet the error budget end to end; marker loss lands the drone and aborts safely; 10 drones fly (scalability) |

## 2. Acceptance criteria (from the MoM)

| Criterion | Limit | How measured |
|---|---|---|
| Position error | <= 10 cm | max |true - reference| during the formation, per drone (`monitoring/metrics.py`) |
| Separation | >= 15 cm edge to edge -> we require 0.514 m centre to centre in formation | min pair distance per tick |
| Synchronisation | all drones in phase | each drone's Z lag behind its own reference (cross-correlation); spread across drones |
| No flight without position feedback | - | fault injection: hide a marker mid-mission |

## 3. Simulation results (full-length missions, 4 drones)

| Task | RMS error | Max error | Min separation | Sync spread | Z lag |
|---|---|---|---|---|---|
| Task 1 - line, travelling height wave | 2.1 cm | 5.3 cm | 0.58 m | 10 ms | 50-60 ms |
| Task 2 - rotating circle, in-phase Z | 1.9 cm | 3.9 cm | 0.97 m | 10 ms | 50-60 ms |
| Hover (closed loop) | 3.8 cm | 6.5 cm | 0.56 m | - | - |

Drift estimates matched the per-drone trim errors injected in the emulator
(e.g. -0.19 m/s^2 on one drone's Y axis), i.e. drift is estimated and cancelled.

## 4. Robustness: simulated drone differs from the controller's model

Two-cycle missions. The controller keeps the nominal plant; only the simulated
drone changes. All runs: no aborts, separation kept.

| Mismatch | Task 1 max error | Task 2 max error |
|---|---|---|
| none | 5.2 cm | 4.0 cm |
| attitude lag 0.25 s (+67 %) | 5.2 cm | 4.1 cm |
| stick gain -25 % | 5.5 cm | 5.3 cm |
| stick gain +25 % | 5.3 cm | 3.8 cm |
| **alt-hold lag 0.5 s (+67 %)** | **9.4 cm** | **9.3 cm** |
| drag x2 | 6.4 cm | 6.3 cm |
| Wi-Fi delay 100 ms +- 30 ms | 5.2 cm | 5.7 cm |
| trim error x3 | 8.5 cm | 6.3 cm |

Conclusion: the alt-hold response is the parameter that matters most (both tasks
are mainly vertical) - it is measured first at Airlink (`hardware_integration.md` §4).

## 5. Fault handling (simulation)

| Fault | Response observed |
|---|---|
| Drone 2's marker hidden mid-circle | 150 ms: level sticks, firmware holds height; 400 ms: drone 2 commanded to LAND; mission aborted; others held 1 s and landed; min separation 0.54 m throughout |
| Drone leaves the camera's view | same as above (tracking LOST) |
| Link drop | firmware lands the drone (RC-loss failsafe); `MspLink` reconnects for telemetry; mission aborts |
| Control code exception | neutral sticks (firmware holds), mission aborted - the RC stream never stops |
| Plan violates separation / fence / limits | refused before take-off continues (e.g. a 10-drone line does not fit the 3 m arena) |

## 6. Bugs found by these tests (kept as regression tests)

- Delayed baro readings erased recent camera fixes during the out-of-sequence
  replay -> filter now buffers measurements and replays all of them
  (`test_delayed_measurements_give_the_same_estimate`).
- During the firmware's take-off climb the filter assumed zero climb rate and
  learned a false 0.9 m/s^2 vertical drift -> measured vario is the vertical input
  whenever the firmware flies the height.
- After a brief tracking dropout the drone resumed FULL control even mid take-off
  -> it now resumes the mode it was in.
- Missing drag feed-forward gave a steady 3 cm lag on the moving circle -> added
  to both controller and filter (error 4.3 -> 1.9 cm RMS).
