# Design

## Goals

- Plan a collision-free route that visits the viewing pose of every obstacle face, within a hard time budget.
- Drive the route through the real STM32 UART protocol, and handle timeouts and faults cleanly.
- Keep the core logic testable without ROS or hardware.

## Key decisions

### Pure C++ cores, thin ROS wrappers
`mdp_planner_core` and `stm32_protocol` contain no ROS includes. The nodes only convert messages and call into them. This keeps unit tests fast, lets the simulator reuse the protocol code, and mirrors how production robotics codebases are structured.

### Planner meets a hard deadline
- Motion primitives: straights F/B and 90° arcs FL/FR/BL/BR using the measured turn radii (24.8 / 33.7 / 33.6 / 24.5 cm), loaded as parameters.
- Collision: the swept robot footprint is sampled about every 1 cm along each primitive, with the offsets pre-computed once per primitive. It is checked against inflated obstacles and the arena walls.
- Legs: A* over a state lattice (binned x, y, heading). Cost = length + turn penalty + reverse penalty.
- Ordering: a pairwise leg-cost matrix, then branch-and-bound over visit orders, pruned by a Held-Karp lower bound.
- Deadline: checked against `std::chrono::steady_clock`. When time runs out, the planner returns the best verified partial route.

### Shared protocol library
The bridge and `virtual_stm32` both use `stm32_protocol`, so encode and parse can't drift between the two sides. `LineFramer` rebuilds full lines from partial serial reads.

### Firmware-faithful simulator
The simulator copies the real firmware's behaviour: one command in flight (`ERR,BUSY` otherwise), exactly one terminal reply per command, priority `STOP`, a motion deadline of 5 s + 0.15 s/cm or 0.10 s/deg, encoders at 75 counts/cm, and an IMU with noise and bias drift. Fault-injection flags (`--drop-rate`, `--imu-fail`, `--stall`, `--noise`) drive the fault tests.

## Failure modes handled

| Failure | Handling |
|---|---|
| No reply from MCU | Bridge timeout → `ERR,TIMEOUT` → mission retries once, then aborts |
| Partial / dropped bytes | `LineFramer` buffers; malformed lines are rejected, never crash |
| Second command while busy | MCU replies `ERR,BUSY`; bridge never sends two at once |
| Serial disconnect | Bridge reports via `/diagnostics` and reconnects |
| Planner out of time | Returns best verified partial route with `partial = true` |
