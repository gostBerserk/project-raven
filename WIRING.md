# Wiring

One identical driver per valve. Repeat for pins 2, 3, 4 and 5.

```
 Arduino pin ──[100-220 ohm]──┬── MOSFET gate (logic-level N-channel)
                              │
                           [10 kohm]
                              │
 Arduino GND ─────────────────┴── MOSFET source ──── common GND ──── 12 V supply GND

 +12 V ──┬── valve (+)
         │
      [1N4007]  cathode (band) to +12 V
         │
         └── valve (-) ──── MOSFET drain
                 (diode anode also to valve (-) / drain)
```

- Arduino GND, the MOSFET sources and the 12 V supply GND must all be connected together.
- The diode goes across the valve coil with the band toward +12 V. Do not leave it out.

## BNO055 (I2C)
| BNO055 | Arduino Uno |
|---|---|
| VIN | 5 V (check your breakout; some are 3.3 V only) |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

Default I2C address is 0x28.

## Valve layout (default in firmware)
| Valve | Arduino pin | Pushes the platform toward |
|---|---|---|
| 0 | D2 | East (0 deg) |
| 1 | D3 | North (90 deg) |
| 2 | D4 | West (180 deg) |
| 3 | D5 | South (270 deg) |

A valve pushes the platform opposite to where its nozzle points, so point the nozzle west for the "east" valve. Edit `VALVE_PUSH_DEG` in the firmware if you mount them differently.
