# Project evaluation

Scores are a judgment of the current repository, not of a finished robot.

| Area | Score | Why |
|---|---|---|
| Idea and scope | 4/5 | Real spacecraft-lab concept, scaled to hobby parts, with a clear pursuit-then-dodge ladder. |
| Software structure | 4/5 | Guidance, navigation and actuation are separate; simulation and camera script share one code path and one parameter file. |
| Verification | 3/5 | 15 unit tests, a fake-camera loop and Monte Carlo sweeps. Nothing has run on hardware yet. |
| Honesty of claims | 5/5 | README labels what is simulated and untested, and reports the result that realistic sensing hurts. |
| Hardware readiness | 2/5 | Parts, wiring and safety docs are good, but thrust, mass and latency are still unmeasured. |
| Documentation | 4/5 | Clear README, build plan and safety notes. |

## Biggest risks, in order

1. **Thrust per valve is unmeasured.** Below about 0.2 N the simulated platform cannot dodge even slow balls.
2. **Position noise.** It was the largest single cause of simulated hits. A bold marker and steady lighting matter more than camera speed.
3. **Untested firmware on real hardware.** Heading sign and IMU calibration are the likely first problems.
4. **Dodge chatter.** Hard switching between dodge and chase has no hysteresis yet.

## Best next steps

1. Build one thruster and measure force with a kitchen scale and a lever. Put the number in the simulation.
2. Measure camera latency (film an LED that toggles on a command) and marker position noise (log a stationary marker for a minute). Put both in the simulation.
3. Run `firmware/valve_test`, then the IMU calibration check, before any closed loop.
4. Try dodge hysteresis and a Kalman filter in the simulation, and keep them only if the 30-run numbers improve on seeds you did not tune on.
