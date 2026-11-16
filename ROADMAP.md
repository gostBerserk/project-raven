# 7-day build roadmap

| Day | Learn | Do | Done when |
|---|---|---|---|
| 1 | PID basics | Rerun the simulation with realistic thrust, tune it, order valves and regulator first | Simulated platform follows the target with few or no hits |
| 2 | Regulators, pressure limits, leak testing | Build the body, mount valves and nozzles, run tubing | Holds pressure at 2 bar or less with no leaks |
| 3 | MOSFET drivers, flyback diodes | Wire 4 valves, run `firmware/valve_test` | Each valve clicks and puffs on command |
| 4 | BNO055 setup and calibration, coordinate frames | Read heading, upload `thruster_controller`, calibrate (watch the `CAL` lines), test the heading sign | "Go east" moves the platform east at any heading |
| 5 | HSV thresholding, pixel-to-table calibration | Track the platform marker, target and balls | Tracking is stable in your real lighting |
| 6 | Python to Arduino over serial | Connect everything, tune the controller | Platform drifts toward a stationary target and stops near it |
| 7 | Closest-approach prediction | Add dodging, roll foam balls, record a demo | Clean video of pursuit plus at least one dodge |

Heads-up: days 4 to 6 (calibration, heading sign, camera lighting) usually take longer than planned, so allow extra days there.

Fallback: if time runs out, ship pursuit only. A working pursuit demo with a clear write-up is still a strong project.

## After week 1
1. Measure real thrust per valve (aim for 0.2 N or more), camera latency and marker position noise, and compensate for differences.
2. Rerun `monte_carlo.py` with measured thrust, mass, latency and noise (see `docs/EVALUATION.md`).
3. Add a Kalman filter to smooth tracking and predict ball paths.
4. Write up the results with photos and video.
