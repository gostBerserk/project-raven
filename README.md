# Project Raven

[![tests](https://github.com/gostBerserk/project-raven/actions/workflows/tests.yml/badge.svg)](https://github.com/gostBerserk/project-raven/actions/workflows/tests.yml)
![status](https://img.shields.io/badge/status-simulation%20%2B%20software%20only-orange)
![license](https://img.shields.io/badge/license-MIT-blue)

Camera tracking, target pursuit and obstacle dodging for small robots that move with puffs of compressed air (cold-gas thrusters). One shared codebase, two vehicles:

1. **Table platform:** a low-friction platform on a flat surface (below).
2. **Raven (drone):** the same tracking and dodging feeding a drone's flight controller, with an optional 6-nozzle cold-gas assist ([details](docs/DRONE.md)).

<p align="center"><img src="docs/images/raven_6_nozzles.png" width="520" alt="Project Raven drone concept with six cold-gas nozzles marked"></p>

*Project Raven concept markup. It is a picture of an idea, not a built or flown drone. The nozzle positions are my estimates.*

**Honest status:** everything here is written, tested in simulation and with unit tests, and nothing has been built, flown or run on real hardware yet. Results labeled "simulation" are model results, not measurements.

---

## Part 1: the table platform

A tabletop mechatronics project: a low-friction platform that uses **compressed-air (cold-gas) thrusters** to chase a colored target and sidestep foam balls rolled at it. An overhead camera tracks everything, a microcontroller fires on/off solenoid valves, and a BNO055 IMU keeps thrust directions correct as the platform turns.

This is the same setup used in university spacecraft labs to practice satellite control on air-bearing tables, scaled down to something a student can build with hobby parts.

![Simulation](results/thruster_sim.gif)

*Demo: 0.6 kg platform, 0.3 N valves, 0.3 to 0.5 m/s balls, realistic sensing (60 ms latency, 8 mm noise, 20 ms valve delay), seed 7. Re-create it with the command under Quick start.*

## Project status

| Part | Status |
|---|---|
| Shared guidance + tracker (`gnc/`) | Done, 38 unit tests pass |
| 2D physics simulation (pursuit + dodging, with latency, noise, valve delay) | Done, runs from the command line |
| Monte Carlo sweeps (ideal vs realistic sensing) | Done (results below) |
| Vision tracker (`vision/track_and_dodge.py`) | Written, passes a fake-camera test, **not yet tested with a real camera** |
| Arduino firmware (`firmware/`) | Written, **not yet tested on hardware** |
| Drone integration (`drone/`): follow + dodge -> flight-controller velocity set-points, 6-nozzle cold-gas assist | Written and simulated, firmware logic checked on a computer. **Never connected to a real flight controller or flown.** See `docs/DRONE.md` |
| Hardware build | **Not started.** Parts list and plan in `docs/` |

Everything below marked "simulation" is a model result, not a measurement from a real platform.

## How it works

The code is split the way guidance, navigation and control (GNC) systems usually are, so each part can be tested on its own:

```
 NAVIGATION (what is where)        GUIDANCE (what to do)         CONTROL / ACTUATION
 vision/ + gnc/estimator.py        gnc/guidance.py               firmware/
 webcam -> HSV blobs ->            closest-approach prediction   "FIRE:<heading>:<ms>" ->
 platform, target, balls ->        dodge sideways beats chase    heading rotation (BNO055) ->
 least-squares velocity fit                                      MOSFETs -> 4 solenoid valves
```

1. **Track:** HSV color thresholding finds the platform marker, the target and the balls. Velocities come from a least-squares fit over the last 10 frames, and ball motion is taken relative to the platform so a moving platform is handled correctly.
2. **Decide:** For each ball, predict the time and distance of closest approach. If a ball will pass too close within about 1.2 s, step sideways (perpendicular to its path, away from where it will pass); otherwise chase the target.
3. **Actuate:** Thrust is on/off, so the Arduino fires short pulses (pulse-width modulation). The IMU heading rotates the table-frame command into the platform's frame so "go east" works at any orientation. The firmware refuses to fire until the BNO055 reports usable calibration.

**The simulation and the camera script run the same guidance and tracker code** (`gnc/`), and share one parameter file (`gnc/params.py`), so a simulation result actually says something about the code that will run on the platform.

## Quick start (simulation)

```bash
pip install -r requirements.txt
python -m unittest discover tests -v        # 38 tests, no camera or hardware needed
cd simulation
python thruster_sim.py --mass 0.6 --thrust 0.3 --ball-speed 0.3 0.5 --seed 7   # GIF + summary plot in ../results
python thruster_sim.py --ideal --no-gif                                         # no latency, noise or valve delay
python monte_carlo.py --runs 30 --thrust 2.0 1.0 0.5 0.2                        # sweep with realistic sensing
python monte_carlo.py --runs 30 --ideal --mass 0.6 --ball-speed 0.3 0.5 --thrust 0.3 0.2 0.1
```

Realistic sensing means 60 ms camera latency, 8 mm position noise and 20 ms valve delay (edit `LATENCY`, `POS_NOISE`, `VALVE_DELAY` in `simulation/thruster_sim.py`). Replace these with what you measure on the real platform.

## Simulation results

30 random runs of 20 s per row (seeds 0 to 29), 2 m x 1.5 m table. The dodge distance and the velocity-fit window were tuned on separate seeds (100 to 129), so these numbers are not tuned on the runs they report. CSVs are in `results/`.

**Baseline: 1.2 kg platform, balls at 0.8 to 1.2 m/s**

| Thrust per valve | Max accel | No ball hit (ideal) | No ball hit (realistic) | Hits per run (realistic) | Target touches per run (realistic) |
|---|---|---|---|---|---|
| 2.0 N | 1.67 m/s² | 67% | 17% | 1.50 | 2.87 |
| 1.0 N | 0.83 m/s² | 27% | 13% | 2.10 | 2.17 |
| 0.5 N | 0.42 m/s² | 10% | 10% | 2.77 | 1.87 |
| 0.2 N | 0.17 m/s² | 0% | 0% | 3.73 | 0.90 |

**Light build: 0.6 kg platform, balls at 0.3 to 0.5 m/s**

| Thrust per valve | Max accel | No ball hit (ideal) | No ball hit (realistic) | Hits per run (realistic) | Target touches per run (realistic) |
|---|---|---|---|---|---|
| 0.3 N | 0.50 m/s² | 77% | 57% | 0.50 | 1.93 |
| 0.2 N | 0.33 m/s² | 70% | 27% | 1.03 | 1.80 |
| 0.1 N | 0.17 m/s² | 20% | 10% | 1.57 | 1.00 |

**What this says**

- **Thrust and mass decide everything.** Acceleration is thrust divided by mass. Published cold-gas air-bearing work reports about 0.1 N per thruster ([Johns Hopkins paper](https://asco.lcsr.jhu.edu/papers/MiKo2014.pdf)), and my rough estimate for a 1 mm nozzle at 2 bar is also 0.1 to 0.3 N (measure yours). The 1.2 kg, 1 m/s baseline is too hard for hobby-scale thrust; the light build is the realistic target: 0.2 to 0.3 N or more per valve, under 0.6 kg, balls rolled slowly.
- **Realistic sensing costs a lot.** At 0.3 N in the light build, "no ball hit" drops from 77% to 57%. In a one-at-a-time test (measured with the earlier 6-frame velocity fit, 30 seeds), position noise alone dropped it from 80% to 50%, while 60 ms latency alone (77%) and 20 ms valve delay alone (77%) barely mattered. So spend effort on a clean marker, good lighting and a good estimator before chasing a faster camera.
- **A longer velocity fit helped.** Moving from 6 to 10 frames was the best of 6, 10, 15 and 20 on the tuning seeds. The gain is modest at 30 runs per row, so treat it as likely rather than proven.
- **Latency extrapolation did not help.** Pushing the estimates forward by 0.04 to 0.12 s gave no measurable gain in the simulation, so it is not included.

## Part 2: Raven (drone)

The same tracking and dodging now runs a drone: `drone/pilot.py` follows a colored target at a standoff distance, sidesteps balls, and sends body-frame velocity set-points to an ArduPilot-style flight controller. It never arms, takes off or changes flight mode, and it is a dry run unless you pass `--connect`. An optional 6-nozzle cold-gas board (`firmware/drone_rcs`, picture in `docs/images/raven_6_nozzles.png`) adds small sideways pulses, but at 0.1 to 0.3 N it is a demonstration extra, not the main actuator. In simulation, follow + dodge left 97% of runs without a ball hit (vs 0% following only), with optimistic assumptions listed in `docs/DRONE.md`. Nothing has flown.

```bash
python drone/sim.py --runs 30                 # drone simulation
python drone/pilot.py --altitude 4 --show     # dry run with a webcam
```

## Repository layout

```
gnc/          params.py (shared constants), estimator.py (velocity fit), association.py (ball tracks), guidance.py (dodge logic)
simulation/   thruster_sim.py (physics + animation), monte_carlo.py (many-run sweep)
vision/       track_and_dodge.py (OpenCV tracking, serial output)
drone/        pilot.py (camera -> flight controller), planner.py, links.py (MAVLink, serial), rcs.py (6-nozzle table), sim.py
firmware/     valve_test/ (bench test), thruster_controller/ (table platform, 4 valves + IMU), drone_rcs/ (drone, 6 valves + ARM switch)
tests/        test_gnc.py (guidance, estimator, simulation), test_drone.py (planner, nozzle table vs firmware, pilot, MAVLink message)
docs/         PARTS.md, WIRING.md, ROADMAP.md, RESOURCES.md, SAFETY.md, EVALUATION.md, DRONE.md, images/
results/      demo GIF, summary plot, Monte Carlo CSVs, drone_sim.txt
```

## Hardware build

See `docs/PARTS.md`, `docs/WIRING.md` and `docs/ROADMAP.md` (a 7-day build plan). Read `docs/SAFETY.md` before connecting compressed air.

Suggested bring-up order: `firmware/valve_test` (valves only) → IMU heading and calibration → camera calibration → full closed loop.

## Known limitations

- The simulation is a 2D point mass. It ignores platform rotation and real surface friction, and it assumes perfect ball-to-track matching and a perfectly known thrust.
- The sensing model is simple (Gaussian position noise, fixed latency, fixed valve delay). Real blob detection also drops frames and mistakes lighting changes for objects.
- Dodge and chase are hard-switched (dodge wins while a threat is predicted), which can chatter near the edge of the danger zone. Hysteresis is untested.
- `track_and_dodge.py` follows a colored marker on top of the platform (blue by default). If the marker is lost for more than 0.3 s it stops firing instead of guessing. Set `USE_PLATFORM_MARKER = False` to assume the platform sits at the image center (desk testing only).
- Ball velocity uses a least-squares fit, not a Kalman filter.
- HSV color ranges and pixel distances are placeholders. Calibrate them under your own lighting and camera height. Red needs two hue ranges because it wraps around the hue circle, so add an orange range if your balls look orange.
- In the firmware, check the sign of `HEADING_SIGN` on the bench. It depends on how the IMU is mounted. Watch the `CAL` lines on the serial monitor and move the platform in a figure-8 until `sys` and `mag` reach 2 or more, otherwise it prints `NOCAL` and does not fire.

## Roadmap

- [ ] Measure real thrust per valve, then re-run the sweep with measured thrust, mass, camera latency and position noise
- [ ] Bench-test valves and firmware
- [ ] Calibrate the camera and add a platform marker
- [ ] Closed-loop pursuit on hardware
- [ ] Add dodging and record a demo video
- [ ] Kalman filter for ball tracking and prediction (replaces the least-squares velocity fit)
- [ ] Dodge hysteresis to stop chatter
- [ ] Per-valve thrust calibration

## Safety

Low-pressure air only (2 bar or less), rated fittings, a pressure relief valve, and eye protection. Details in `docs/SAFETY.md`. This is a classroom-scale robotics project and nothing in it is designed to carry or hit anything but soft foam balls.

## Contributing

Issues and pull requests are welcome, especially thrust measurements, camera latency and noise numbers from real builds, and hardware test reports. Run `python -m unittest discover tests` before opening a pull request. See `CONTRIBUTING.md`.

## License

The code and documents are MIT licensed, see `LICENSE`.

**Image note:** the drone picture in `docs/images/` is based on a third-party 3D render. The MIT license does not cover it. Source and permission: **[fill in before publishing, or replace it with your own render or photo]**.
