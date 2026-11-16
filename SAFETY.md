# Safety

- **Pressure:** run at 2 bar or less. Fit a regulator with a gauge and a pressure relief valve set below the lowest rating of any component.
- **Components:** use only tubing and fittings rated above your working pressure. Never use damaged bottles or containers as tanks.
- **Eye protection** whenever the system is pressurized.
- **Leak test** at low pressure first, and depressurize before changing any fitting.
- **Electrical:** valves are inductive loads, so always fit the flyback diode and share a ground between the Arduino, MOSFETs and supply.
- **Firmware:** pulses are time-limited, and all valves stay closed at boot. Keep that behavior if you modify the code.
- **Test area:** use foam balls only, keep people out of the platform's path, and keep the table edge clear so nothing falls.
