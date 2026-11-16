# Parts list

Prices vary a lot by supplier, so check locally. Order the valves and regulator first, since they take longest.

## Pneumatics
| Qty | Part | Notes |
|---|---|---|
| 4 | 12 V normally-closed 2-way solenoid valves, push-fit | Small, fast, low flow |
| 4 | Nozzles or push-fit fittings with ~1 mm opening | The opening sets thrust |
| 1 | Airbrush compressor with tank | Keep off the platform; use a light hose |
| 1 | Pressure regulator with gauge | Set to 2 bar or less |
| 1 | Pressure relief valve | Safety, set below the lowest component rating |
| - | 6 mm PU tubing, T-manifold, push-fit connectors | Rated for the working pressure |

## Electronics
| Qty | Part | Notes |
|---|---|---|
| 1 | Arduino Uno (or ESP32) | |
| 1 | BNO055 IMU breakout | Gives fused heading |
| 4 | Logic-level N-channel MOSFETs (e.g. IRLZ44N) | Not a non-logic-level part like IRFZ44N |
| 4 | Flyback diodes (1N4007) | One across each valve |
| 4 | 100-220 ohm gate resistors and 10 kohm pull-downs | |
| 1 | 12 V 3 A power supply | Tethered |
| 1 | USB webcam and overhead mount | |
| - | Breadboard, jumper wires | |

## Body and surface
- 25 cm disc or square body (3D-printed, foam board or acrylic), as light as you can make it
- PTFE glide pads under the body
- Large sheet of glass or acrylic, or a very smooth table
- Colored stickers or balls: one color for the target, another for the obstacles (red or orange is fine, see the README note on hue)
- A third color (e.g. blue) marker, flat and about 5 cm across, on top of the platform so the camera can find it
- Soft foam balls

## Tools
Screwdrivers, hot glue, zip ties, multimeter, safety glasses.
