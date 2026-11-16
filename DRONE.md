# Drone integration

The tracking and dodging code from the table platform now drives a drone. **Nothing here has flown or been connected to a real flight controller yet.** The results below are simulation results.

## What it does

```
 downward camera ──> drone/pilot.py ──> body-frame velocity set-point ──> flight controller (props do the moving)
                     │  HSV blobs -> metres (height + field of view)
                     │  gnc/association.py  tracks balls, estimates velocity
                     │  drone/planner.py    FOLLOW target at a standoff, DODGE balls (gnc/guidance.py)
                     └──> optional "FIRE:<heading>:<ms>" ──> 6-nozzle cold-gas board (firmware/drone_rcs)
```

- **Follow:** keeps a colored target (for example a person carrying a green marker) at a standoff distance, default 2 m. It never closes in: inside the standoff distance it backs off.
- **Dodge:** if a ball will pass within the danger radius within 1.2 s, it sidesteps. Dodging has priority over following.
- **Same code as the simulation and the table platform:** `gnc/guidance.py` and `gnc/association.py`.

## The 6 nozzles

See `docs/images/raven_6_nozzles.png`. `firmware/drone_rcs/drone_rcs.ino` and `drone/rcs.py` hold the same table (a test checks they match). Heading is in the body frame: 0 = forward (the side with the PROJECT RAVEN name), 90 = left.

| Valve | Position | Exhausts | Pushes the drone | Arduino pin |
|---|---|---|---|---|
| 1 | Nose, left | left | right (270°) | 2 |
| 2 | Nose, right | right | left (90°) | 3 |
| 3 | Tail, left | left | right (270°) | 4 |
| 4 | Tail, right | right | left (90°) | 5 |
| 5 | Front face, centreline | toward the front | back (180°) | 6 |
| 6 | Back face, centreline | toward the back | forward (0°) | 7 |

**Why valves 5 and 6 are on the centreline:** a push that misses the centre of mass also twists the drone. Mounted on opposite wings, 5 and 6 push in opposite directions and make a pure spin with no net push. On the centreline, the push passes through the middle, so there is no spin. `tests/test_drone.py` checks that no heading produces yaw torque. Nozzle positions in `drone/rcs.py` are estimates from the render, so measure yours.

A heading fires the best valve plus any within 50° of it, so 90° opens valves 2 and 4 together, and 45° opens 2, 4 and 6. Wire the valves the way `docs/WIRING.md` describes (logic-level MOSFET and flyback diode per valve, common ground, separate valve supply). The ARM switch goes between pin 8 and ground: **valves can only open while it is closed**, and opening it shuts every valve at once.

**The cold-gas assist is small.** At 0.1 to 0.3 N on a drone of about 1.5 kg, one nozzle gives roughly 0.07 to 0.2 m/s². The simulation assumes the propellers can give about 4 m/s², so the cold gas is a demonstration extra, not the thing that dodges. It is left out of the drone simulation for that reason.

## Assumptions

- Downward-facing camera, level, image-up = the drone's forward direction, flat ground, known height above it (from the flight controller if it reports it, otherwise `--altitude`).
- The drone holds its heading (no yaw) while following.
- The program is written for an ArduPilot-style flight controller in GUIDED mode (`SET_POSITION_TARGET_LOCAL_NED`, body offset frame, velocity only). PX4 needs offboard-mode changes. A tail-sitter like this one behaves like a multicopter in hover only, so use hover-mode flight.

## Safety design

- **The program never arms, takes off, lands or changes mode.** You do that with the RC transmitter. Commands are sent only while the vehicle is armed and in GUIDED, and switching out of GUIDED takes control back immediately.
- Speed is capped (default 1.5 m/s). It sends a zero-velocity command if the camera stops delivering frames, and on exit.
- Default is a **dry run** that sends nothing.
- Before real flights: read your country's current drone rules (in India, check the DGCA / Digital Sky requirements for registration, permissions and where you may fly), keep away from people, and use prop guards.

## Bring-up order

1. **Dry run with a webcam:** `python drone/pilot.py --altitude 4 --show`. Hold the camera by hand, walk a green marker through the view and roll a red ball past. Check the printed `FOLLOW` / `DODGE` directions.
2. **SITL:** run ArduPilot SITL, then `python drone/pilot.py --connect udp:127.0.0.1:14550`, and arm and switch to GUIDED in SITL. This tests the MAVLink path only; SITL has no camera.
3. **Valve bench test** with the drone unpowered and propellers removed, using the ARM switch.
4. **Dodging with the camera on a cart or held by hand**, before the drone flies with the program in the loop.
5. **Tethered or netted hover** with the RC transmitter in your hand, then wider flights where the rules allow.

## Simulation results

`python drone/sim.py --runs 30` gives these. 30 runs of 20 s (seeds 0 to 29), nothing tuned on them. The drone is a velocity-commanded point (0.3 s response, 4 m/s² limit) at 4 m height with an 80° lens, balls thrown at 1 to 2.5 m/s from 7 m, 80 ms latency and 2 cm position noise. Raw output is in `results/drone_sim.txt`.

| Case | Runs with no ball hit | Hits per run |
|---|---|---|
| Follow only (no dodge) | 0% | 3.40 |
| Follow + dodge, realistic sensing (max 1.5 m/s) | 97% | 0.03 |
| Follow + dodge, ideal sensing | 100% | 0.00 |
| Follow + dodge, max speed 1.0 m/s | 93% | 0.07 |
| Follow + dodge, max speed 2.5 m/s | 100% | 0.00 |

Mean distance error from the 2 m standoff stays at about 0.5 m while following a person-speed target.

**Read these numbers with care.** They are much better than the table platform's because a propeller drone accelerates far harder than 0.2 N of cold gas can. They are also optimistic: the model has no wind, tilt, yaw, rolling shutter, motion blur or missed detections, it knows the height exactly, and it assumes the drone's response is 0.3 s with 4 m/s², which is my guess for a small quad. The speed cap matters: 1.0 m/s lets a few balls through.

## Not done yet

- Real flight controller and camera tests.
- Camera tilt compensation (a tilted drone shifts the image) and wind.
- A Kalman filter for ball tracking.
- PX4 support and a forward-flight mode for the tail-sitter.
