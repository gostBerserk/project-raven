# Contributing

Thanks for looking at Project Raven. This is a student project, so small, clear changes are best.

- **Run the tests:** `python -m unittest discover tests -v` (no camera or hardware needed).
- **Keep the simulation honest:** if you change guidance or tracking, re-run the Monte Carlo sweeps on seeds you did not tune on, and update the README tables.
- **Hardware reports are the most useful thing:** measured thrust per valve, camera latency, marker position noise, and anything that broke. Please say what you built and measured, and keep simulation results and hardware results separate.
- **Safety:** compressed air and flying machines can hurt people. Do not remove the ARM switch, speed cap or dry-run default. Read `docs/SAFETY.md` and `docs/DRONE.md`.
