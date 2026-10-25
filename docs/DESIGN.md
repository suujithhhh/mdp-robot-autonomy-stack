# Design document

Status: prototype. Everything in this document marked **[verified]** is covered by an automated
test or a measurement in this repository; anything marked **[unverified]** has not been run end to end
(the ROS 2 nodes compile in CI but have never been launched; see README, "What is and is not verified").

## 1. Problem and goals

The NTU SC2079 MDP robot (STM32F407, FreeRTOS, Raspberry Pi, Android, a PC-side Python path
planner) placed 1st of 32 teams in Task 1. Its software was split across languages and tightly
coupled to the hardware, so the planner and the driver code could not be tested without the robot.

Goals of this re-engineering:

| # | Goal | How it is met |
|---|---|---|
| G1 | Port the planner to modern C++ with *identical* behaviour | Operation-for-operation port; compared leg by leg against the original (`tools/compare.py`) |
| G2 | Make the planner fast and bounded in time | A* lattice search, cached leg results, Held-Karp pruned ordering, hard deadline with best-verified-partial fallback |
| G3 | Test the robot-side protocol and driver without hardware | `virtual_stm32`, a firmware-faithful simulator, shares the protocol library with the driver |
| G4 | Prove the system fails safe | Fault injection: lost bytes, stalled motors, IMU failure, wheel slip, STOP mid-move |
| G5 | Run it as a ROS 2 system | Thin ROS 2 nodes around the tested libraries **[unverified]** |

Non-goals: image recognition, the Android app, Task 2 (IR/ultrasonic parking), controlling real
hardware, dynamic obstacle avoidance.

## 2. Architecture

```
 ┌───────────────┐ commands ┌─────────────────┐  wire strings  ┌──────────────────┐ bytes ┌───────────────┐
 │ mdp_planner   │─────────▶│ mdp_mission     │───────────────▶│ stm32_bridge     │◀═════▶│ virtual_stm32 │
 │ (pure C++)    │  route   │ (pure C++)      │  BridgeCore    │ (pure C++ core)  │ UART  │ (pure C++)    │
 └───────────────┘          └─────────────────┘                └──────────────────┘       └───────────────┘
        ▲                          ▲                                   ▲                         ▲
        └────────────── all four depend on stm32_protocol / mdp_planner_core (pure C++) ────────┘

 ROS 2 layer [unverified]:  mdp_planner_node · mission_node · stm32_bridge_node   (thin wrappers)
```

**Key decision: logic in pure C++ libraries, ROS only at the edges.**

* The libraries have no ROS or OS dependency, so they build in seconds, run in plain GTest, and
  run in CI without a ROS install.
* The ROS nodes translate between topics/services and library calls and contain no decisions.
* The driver core never touches a serial port: bytes and time are passed in. The same class is
  driven by the ROS node (real clock, real serial) and by tests (simulated clock, in-memory wire).
  This is what makes a 14-second robot mission run in ~100 ms of test time and be deterministic.

| Package | Responsibility |
|---|---|
| `mdp_planner_core` | Footprint + SAT collision, six motion primitives with four measured turn radii, swept-path collision, lattice A*, command compression, viewing poses, route search with Held-Karp bound |
| `stm32_protocol` | Command/reply encode + parse, strict validation, `LineFramer` |
| `virtual_stm32` | Device model: state machine, motion timing, encoders, IMU, STOP, deadline, faults |
| `stm32_bridge` (`BridgeCore`) | Host protocol state machine, timeouts, dead-reckoning + wheel/IMU odometry |
| `mdp_mission` | Executes a route command by command; retry/abort policy; `SimLink` test transport |

## 3. Planner

Faithful port of the Python planner's executable pipeline (algo/ of the MDP repo):

1. **Primitives** `F B FL FR BL BR` propagate a rear-axle pose along exact constant-radius arcs.
   The four turn radii (24.8 / 33.7 / 33.6 / 24.5 cm) are independent measured parameters.
2. **Swept collision**: the margin-inflated footprint is checked at samples along the whole
   primitive, with an anti-tunnelling guard (half an interval of footprint travel) so clearance
   holds *between* samples too. Obstacle rectangles are built once per leg.
3. **Lattice A\***: state key = (5 cm position bin, 10° heading bin, previous command). Cost =
   length + reverse penalty + a fixed cost per *new* command (so few long commands beat many
   short ones). The heuristic is the max of a distance bound and a rotation bound, plus the cost
   of one more turn command; both are admissible.
4. **Compression** merges runs of equal commands (`F10 F10 F10 → F30`), re-validating the merged
   swept path and replaying it against the original, within 1e-6 cm.
5. **Route ordering**: DFS over visiting orders with a per-leg cache keyed by the exact pose
   pair. A subtree is pruned when `cost so far + HeldKarp(remaining) ≥ incumbent`.
6. **Deadline**: every leg search polls a `steady_clock` deadline. On expiry the route returns
   the best *verified* route found so far (complete if any, else the longest verified prefix),
   flagged `timed_out`. Timeouts are never cached.

### Held-Karp bound (admissibility)

A leg ends within the goal tolerance `τ` of target `a` and must reach within `τ` of target `b`,
so its length is ≥ `dist(a,b) − 2τ`, and its cost ≥ length · min(1, reverse_penalty) since every
primitive costs at least that and command costs are non-negative. From the *actual* pose `p` the
first leg is ≥ `dist(p,b) − τ`. A Held-Karp DP over the pairwise bounds gives the cheapest
possible way to visit the remaining set; any real completion costs at least that. Because it
never overestimates, pruning cannot discard the optimal route.
**[verified]** by `HeldKarp.BoundNeverExceedsTrueShortestVisitOrder` (brute force) and
`Route.HeldKarpPruningDoesNotChangeTheOptimalCost`.

### Floating-point identity

The port reproduces the reference operation for operation (same expression order, Python's
banker's `round()` via `nearbyint`, Python's float `%` semantics, CPython's `radians`/`degrees`
constants) and the build disables FMA contraction (`-ffp-contract=off`). This is why the C++ and
Python planners can be compared for *exact* equality of costs and command lists rather than
within a tolerance.

## 4. Protocol and driver

Contract (from the MDP `PROTOCOL.md`): ASCII lines, ≤31 characters host→device, exactly one
command in flight, terminal replies `ACK`, `ERR,<code>`, `ACK,STOPPED`; everything else is
telemetry and never a reply; no command IDs.

Driver rules **[verified]** in `test_bridge_core.cpp`:

* A second command while one is in flight is refused *locally* (never sent).
* No terminal reply by `firmware_deadline + margin` → `LinkTimeout`, pose marked **uncertain**,
  a confirming `STOP` is sent, and the command is **never resent** (without IDs a resend could
  double a physical move).
* `ERR,MOTION_TIMEOUT` and a `STOP` that crossed a movement mark the pose uncertain;
  `ERR,IMU_NOT_READY` before `MM,EV:START` does not (nothing moved, so one retry is allowed).
* `ACK,STOPPED` is never treated as movement success; unexpected terminal replies are counted,
  not applied.
* Lines longer than the limit are discarded and the framer resynchronises at the next newline.

Odometry: wheel-count deltas at ~10 Hz along the IMU heading (midpoint integration), corrected
at the end of each movement with the `DL/DR` totals, and the heading refreshed with `IMU,GET`
at rest after each ACK. The ACK-based *commanded pose* is kept separately, because ACK confirms
completion, not position — the two diverge under wheel slip, which is intentional and tested.

### Bug found by the tests (kept as a lesson)

The host framer originally reused the 31-character limit of the *command* direction for incoming
lines. Telemetry (`MM,TEL,…`) and `MM,EV:END,…` lines are longer, so they were silently dropped
and odometry never updated. The unit tests for the end-to-end odometry caught it; the limit is
now a per-direction constructor argument (`kMaxLineChars` for commands, `kMaxHostLineChars` for
incoming lines).

## 5. Failure modes

| Fault | Behaviour | Test |
|---|---|---|
| Device silent / bytes lost | `LinkTimeout`, pose uncertain, confirming STOP, no resend | `EndToEndFaults.TotalSilence…`, `LostCommandBytes…` |
| Motors stalled | `ERR,MOTION_TIMEOUT` at the firmware deadline; mission aborts, pose uncertain | `StalledMotors…` |
| IMU not ready | one retry if nothing moved, then abort with the pose still known | `ImuFailure…` |
| Random byte loss, 20 seeds | always terminates in bounded time with a definite outcome | `RandomByteLoss…` |
| Wheel slip | commands still ACK; odometry sees the drift, commanded pose does not | `WheelSlip…` |
| STOP mid-move | `ACK,STOPPED` only, robot stationary, pose uncertain | `StopMidMission…` |
| Planner runs out of time | best verified route returned, flagged `timed_out` | `Route.TimeLimit…` |
| No safe viewing pose | fails fast with `no_safe_viewing_pose` | `Route.ObstacleWithNoSafe…` |

## 6. Trade-offs

* **Exact port vs redesign.** An exact port was chosen so the C++ planner can be *proven*
  equivalent to the validated one. Improvements (e.g. a better lower bound, or reusing the
  lattice across legs) are deliberately deferred until equivalence is established.
* **Prefix-tree search not ported.** The Python v2 service uses a prefix-tree branch-and-bound
  inside a contained worker process. The C++ route search is a plain DFS with the same pruning
  idea and an in-process deadline; it is simpler, but has not been compared at 6–8 obstacles
  against the Python bounded executor.
* **Simulator fidelity.** `virtual_stm32` uses timings measured on the robot but constant-rate
  motion, a nominal wheel track, and simple noise models. It tests protocol and driver
  behaviour, **not** control-loop quality; it is not a substitute for the real robot.
* **Single-threaded search.** Simpler and deterministic; parallel leg planning is possible later.

## 7. Test strategy

* Unit (GTest): primitives, collision, lattice, compression, protocol, framer, device model,
  driver state machine, route search with an injected fake leg planner.
* Parity: `tools/compare.py` (leg level, all benchmark fixtures) and `tools/compare_routes.py`
  (route level, small layouts) against the original Python planner. See `docs/BENCHMARK.md`.
* End-to-end (GTest, in memory): real planner → mission executor → driver → virtual STM32, with
  and without faults.
* CI (`.github/workflows/ci.yml`): build + test (Release/Debug), AddressSanitizer + UBSan,
  clang-format, ROS 2 Humble colcon build. **[verified]**: all jobs are green on `main`.
