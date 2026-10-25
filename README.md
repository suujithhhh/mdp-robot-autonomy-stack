# MDP Robot Autonomy Stack (C++17, ROS 2)

A C++17 re-engineering of the software from my NTU SC2079 Multidisciplinary Design Project
robot (1st of 32 teams, Task 1): the path planner, the STM32 serial protocol and driver, and a
firmware-faithful STM32 simulator, so the whole stack can be developed and tested without the
robot. ROS 2 nodes wrap the tested libraries.

> **Honest status.** This is a working prototype. The libraries, the simulator and the
> end-to-end tests are built and passing (141 GTest cases). The ROS 2 layer, the Linux
> pseudo-terminal front end, CI and the sanitizer runs are written but **not yet built or run**
> (development so far was on Windows). See "What is and is not verified".

## Architecture

```
 mdp_planner ──route──▶ mdp_mission ──"MOV,FL,90"──▶ stm32_bridge ◀══ UART bytes ══▶ virtual_stm32
 (lattice A*,          (one command at a time,       (state machine, timeouts,        (device model: ACK/ERR,
  Held-Karp B&B)        retry / abort policy)         odometry from wheels + IMU)      STOP, deadline, faults)
        └──────────── all depend on pure C++ libraries: mdp_planner_core, stm32_protocol ────────────┘
```

Logic lives in pure C++ libraries with no ROS or OS dependency; the ROS 2 nodes
(`ros2/nodes/`) are thin wrappers. The driver core takes bytes and a clock as arguments, so the
same class runs on a real serial port (ROS node) or against the simulator in a deterministic,
simulated-time test. Details: [`docs/DESIGN.md`](docs/DESIGN.md).

| Package | What it is |
|---|---|
| `src/mdp_planner_core` | Footprint + SAT collision, 6 motion primitives (4 measured turn radii), swept-path collision, lattice A\*, command compression, route search with Held-Karp pruning and a hard deadline |
| `src/stm32_protocol` | Command/reply encoding, strict parsing, line framing |
| `src/virtual_stm32` | Device model + a Linux pseudo-terminal front end |
| `src/stm32_bridge` | Driver core: protocol state machine, timeouts, odometry |
| `src/mdp_mission` | Mission executor + in-memory `SimLink` used by the tests |
| `ros2/` | ROS 2 nodes, launch file (not yet built) |
| `tools/` | Python parity + benchmark harness against the original planner |

## Build and test (no ROS needed)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure      # 141 tests
./build/src/mdp_mission/mdp_sim_mission          # plan + drive a 3-obstacle mission in simulation
```

Requires CMake ≥ 3.16, a C++17 compiler and GoogleTest. Floating-point contraction is disabled
(`-ffp-contract=off`) so results match the Python reference bit for bit.

## Results

Measured against the original, unmodified Python planner on the 33 benchmark layouts from the
MDP repository — full tables in [`docs/BENCHMARK.md`](docs/BENCHMARK.md), reproducible with
`tools/compare.py`:

| Check (33 layouts, 327 legs) | Result |
|---|---|
| Same valid/invalid verdict as Python | 327 / 327 |
| Identical command lists on legs both planners solve | 270 / 270 (cost difference 0) |
| Legs both reject, for the same reason | 57 / 57 |
| Planning speed-up, whole benchmark | **33.1x** (median per leg 31.6x; clean serial re-check 33.2x) |
| 4-obstacle route, exhaustive Python vs C++ | 359 s and 684 s vs 11 s and 21 s, same optimal route |
| 6-8 obstacles (C++ only) | hits the 70 s limit, returns a complete verified route (not proven optimal) |

## What is and is not verified

| Item | Status |
|---|---|
| Planner core, protocol, simulator, driver, mission executor | Built; 141 GTest cases pass on Windows (MSYS2 GCC 13) |
| C++ vs Python planner equivalence | Measured leg by leg, see BENCHMARK.md |
| Fault handling (silence, stall, IMU failure, byte loss, slip, STOP) | Covered by end-to-end tests with the simulator |
| ROS 2 nodes + launch file | Written, **not compiled** (needs Ubuntu + ROS 2 Humble) |
| `virtual_stm32_pty` | Written, **not compiled** (Linux only) |
| GitHub Actions CI (incl. ASan/UBSan, clang-format, colcon) | Written, **never run** |
| clang-tidy | Not configured yet |
| Behaviour on the physical robot | **Not tested** — the simulator covers protocol and driver logic, not control-loop quality |

## Known limitations

* The route search is a plain DFS with Held-Karp pruning and a wall-clock deadline. For 6–8
  obstacles it normally stops at the time limit and returns the best *verified* route found, which
  is not proven optimal. The Python v2 service's prefix-tree executor is not ported.
* Per-leg search cost is dominated by lattice A\*; planning one leg from an arbitrary pose can
  take hundreds of milliseconds.
* The simulator uses constant-rate motion and a nominal wheel track; see the header of
  `virtual_stm32.hpp` for every assumption.
