# Test report

Run: `cmake -S . -B build && cmake --build build && ctest --test-dir build` (MSYS2 UCRT64 GCC 13.1,
CMake 3.26, GoogleTest from the MSYS2 package). Result: **141 / 141 passed**.

| Suite | Cases | What it covers |
|---|---|---|
| `mdp_planner_core_tests` | 49 | Primitive geometry (worked example from PROTOCOL.md, per-direction radii, turn sense, full circle, value limits), footprint + SAT collision (margin applied once, wall touching, rotated body), swept collision (obstacle *between* endpoints, wall sweep, sample spacing), lattice search (unsafe start/goal, replay reaches goal, every sample collision-free, expansion limit, deadline), compression (cost + endpoint preserved, splitting above the 999 cm limit), viewing poses, Held-Karp bound vs brute force, route search with an injected fake leg planner (optimal order, pruning equivalence, cache, partial route, deadline) |
| `stm32_protocol_tests` | 21 | Encode/parse round trip, every STOP variant rejected, malformed MOV lines, value limits, 31-char limit, reply classification (terminal vs telemetry), telemetry round trip, `LineFramer` (split reads, CRLF, byte-at-a-time, overflow resync, bounded memory) |
| `virtual_stm32_tests` | 25 | One ACK per move, ideal end pose, START/END bracketing, BUSY, BAD_CMD, 10 Hz telemetry, encoder counts (75/cm, reverse negative, inner/outer wheel), STOP while moving / idle / duplicate / during STOP, fault injection (stall → MOTION_TIMEOUT, IMU fail, byte loss, slip, IMU bias, seeded noise) |
| `stm32_bridge_tests` | 26 | One command in flight, ACK/ERR mapping, pose-uncertainty rules, telemetry never a reply, unexpected/garbage lines counted, partial reads, link timeout → confirming STOP and **no resend**, STOP handling, and the driver against the virtual device (odometry vs truth on straights and turns, slip, byte-at-a-time, stall, total silence, STOP mid-move) |
| `mdp_mission_tests` | 20 | Real planner → executor → driver → virtual STM32: full 3-obstacle mission, every viewing pose within the 3 cm goal tolerance, odometry vs truth, 1-byte reads give the same result, the robot never enters an obstacle while driving, stalled motors, IMU failure + retry budget, total silence, lost command bytes (command sent exactly once), 20 random byte-loss seeds all terminate with a definite outcome, wheel slip, IMU bias, STOP mid-mission |

## Defects found by the tests

1. **Telemetry silently dropped (real bug, fixed).** The host-side line framer applied the
   31-character *command* limit to incoming lines. `MM,TEL,…` and `MM,EV:END,…` are longer, so
   odometry never updated. Found by the end-to-end odometry tests; the limit is now per direction.
2. **Heading lag at the end of a turn (real accuracy gap, fixed).** The last <100 ms of a movement
   is not covered by 10 Hz telemetry, leaving ~0.7° heading error after a 90° turn. The driver
   now refreshes the heading with `IMU,GET` at rest after each ACK (as the Pi does before a turn).
3. Four *test* mistakes (reversed BL/BR turn-direction expectation; goals whose nose left the
   arena; two route-test layouts with an off-arena camera). The code was right in all four cases;
   the expectations were corrected, not the implementation.

## Continuous integration (GitHub Actions, Ubuntu 22.04)

All jobs pass on `main`: build + `ctest` in Release and Debug, the full suite under
AddressSanitizer + UndefinedBehaviorSanitizer, `clang-format --Werror`, and a ROS 2 Humble
`colcon build` + `colcon test`. The first CI run failed only the formatting check; the code was
reformatted with no functional change and the 141 tests were re-run.

## Not covered / not yet run

* The ROS 2 nodes (`ros2/`) and `virtual_stm32_pty` (Linux pseudo-terminal front end) **compile**
  in CI (Ubuntu 22.04, ROS 2 Humble) but have **not been run**: no node has been launched and the
  pseudo-terminal path has not been exercised.
* `clang-tidy` is not configured.
* The simulator tests protocol and driver behaviour, not control-loop quality; nothing here has
  been run against the physical robot.
