# Synchronized Multi-Drone Swarm Execution

**Inter IIT Tech Meet 15.0 | Drona Aviation**

> Development repository for a multi-drone swarm platform using Pluto drones. This repository is a project scaffold; implementation and test results will be added as the team develops and validates them.

## Project overview
Develop a platform for coordinated execution of predefined drone formations and synchronized movements, with trajectory generation, external vision-based localization, real-time monitoring, and positional drift correction.

## Required swarm tasks
1. **Line formation with wave effect:** arrange four drones in a straight line and execute a sequential wave-like movement while maintaining the line.
2. **Circular formation with wave effect:** arrange four drones in a circular formation and execute a coordinated wave-like movement while maintaining formation geometry and safe separation.

Both tasks must be validated in simulation and demonstrated on physical Pluto drones.

## Main capabilities to develop
- Drone identification and swarm configuration
- Formation generation and synchronized trajectory planning
- Position and altitude control, subject to platform capabilities
- Collision detection, separation checks, and pre-flight validation
- External vision-based localization and coordinate transformation
- Closed-loop positional drift correction
- Live monitoring, visualization, and flight-data logging
- Simulation and physical-hardware validation

## Repository structure
- `config/` — example configuration files
- `docs/` — project, setup, architecture, and testing documentation
- `reports/` — final technical and performance reports
- `scripts/` — helper and launch scripts
- `simulation/` — simulation scenarios and experiments
- `src/` — application source code, separated by responsibility
- `tests/` — unit, integration, and hardware tests
- `results/` — simulation and physical-test outputs

## Getting started
Setup and run instructions will be added after the team confirms the supported ROS 2 environment, Pluto interfaces, external positioning setup, and dependencies. See `docs/setup_guide.md` for the current setup checklist.

## Hardware and software references
See [`docs/references.md`](docs/references.md) for the resources listed in the problem statement. Use the provided Pluto ROS 2 package, Magis V2 firmware, and PlutoControl library as the primary hardware interfaces. Confirm versions and compatibility before pinning dependencies.

## Safety note
Do not run physical drone tests until the operating area, drone separation, localization validity, communication status, and pre-flight checks have been reviewed. Never commit credentials, private keys, drone IP details, or personal access tokens. Keep secrets in local environment variables or an untracked configuration file.

## Team
Add team members and their responsibilities here.

## Project status
- [ ] Repository and development workflow
- [ ] Simulation environment
- [ ] Line formation with wave effect
- [ ] Circular formation with wave effect
- [ ] External vision localization
- [ ] Drift correction and monitoring
- [ ] Safety checks and fault handling
- [ ] Physical Pluto drone validation
- [ ] Docker deployment package
- [ ] Technical and performance reports

## Deliverables
The problem statement specifies a GitHub source-code repository, physical demonstration of both tasks, a technical documentation report (maximum 25 pages excluding cover and references), and a performance/testing report (maximum 10 pages). The stated submission deadline is **5 December 2026**.
