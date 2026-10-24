# MDP Robot Autonomy Stack

> 🚧 **In progress:** active development, Oct 2026 – present

A ROS 2 (C++17) autonomy stack for a car-like robot. It has a time-bounded state-lattice planner, a ROS 2 serial driver, and a firmware-faithful STM32 simulator that speaks the real UART protocol. Everything is tested with GTest and end-to-end launch tests in GitHub Actions CI.

It re-engineers the software from my [NTU SC2079 MDP robot](https://github.com/suujithhhh/SC2079-MDP-Group7), which was tightly coupled to its hardware, into a modular stack. The algorithm and protocol logic live in **pure C++ libraries with no ROS dependency**, and the ROS nodes are thin wrappers around them. The core can therefore be unit-tested quickly without ROS, and reused elsewhere.

## Architecture

```
 ┌──────────────┐  PlanRoute   ┌──────────────────┐  SendCommand  ┌──────────────┐  serial (pty)  ┌───────────────┐
 │ mdp_planner  │◀────action───│ mission_executor │─────action───▶│ stm32_bridge │════════════════│ virtual_stm32 │
 │ (ROS 2 node) │──route──────▶│ (ROS 2 node)     │◀──ACK/ERR─────│ (ROS 2 node) │  MOV / ACK /   │ (plain C++,   │
 └──────┬───────┘              └──────────────────┘               └──────┬───────┘  ERR / TEL     │  no ROS)      │
        │ uses                                                           │ uses                   └───────────────┘
 ┌──────▼────────────┐                                            ┌──────▼─────────┐                    │ uses
 │ mdp_planner_core  │  pure C++ library, no ROS                  │ stm32_protocol │◀───────────────────┘
 └───────────────────┘                                            └────────────────┘  pure C++ library, shared
        Publishes: /planned_path, /obstacle_markers          Publishes: /odom, /imu, /tf, /diagnostics
```

## Packages

| Package | Type | Purpose |
|---|---|---|
| `mdp_interfaces` | ROS 2 | `Obstacle.msg`, `PlanRoute` / `SendCommand` / `ExecuteMission` actions |
| `mdp_planner_core` | C++ lib | State-lattice A*, swept-footprint collision, branch-and-bound visit ordering with a Held-Karp lower bound, hard deadline with best-partial fallback |
| `mdp_planner` | ROS 2 node | `PlanRoute` action server wrapping the core |
| `stm32_protocol` | C++ lib | UART command encode/parse, reply parsing, `LineFramer` for partial serial reads. Shared by the bridge and the simulator so they can't drift apart |
| `virtual_stm32` | C++ exe | Pseudo-terminal STM32 simulator: one command in flight, priority `STOP`, motion timeouts, bicycle kinematics with measured turn radii, 10 Hz telemetry, fault injection |
| `stm32_bridge` | ROS 2 node | termios serial driver, command state machine, `/odom` `/imu` `/tf` `/diagnostics` |
| `mdp_mission` | ROS 2 node | Plans, then executes the route command by command with retry/abort |
| `mdp_bringup` | Launch | One-command sim launch, URDF, RViz config, end-to-end tests |

## Testing strategy

| Level | Tool | Covers |
|---|---|---|
| Unit | GTest | Primitives, collision edge cases, A*, Held-Karp bound, B&B, protocol, `LineFramer`, bridge state machine |
| Property | GTest | Random layouts: every route is collision-free and within the deadline |
| End-to-end | `launch_testing` | Full stack runs a 6-obstacle mission within pose tolerance |
| Fault | `launch_testing` | Injected timeouts, dropped bytes, IMU failure → clean `ERR` handling, no hangs |
| Static / runtime | clang-tidy, ASan, UBSan | Code quality and memory safety in CI |

## Roadmap

- [x] Architecture and design ([docs/DESIGN.md](docs/DESIGN.md))
- [ ] Workspace, package skeletons, CI on Ubuntu 22.04 / ROS 2 Humble
- [ ] `mdp_planner_core`: types, motion primitives, collision checking
- [ ] A* leg search, viewing poses
- [ ] Visit ordering (B&B + Held-Karp), deadline, command compression
- [ ] `stm32_protocol` + `LineFramer`
- [ ] `virtual_stm32` simulator with fault injection
- [ ] `stm32_bridge` serial driver node
- [ ] `mdp_planner` and `mdp_mission` nodes
- [ ] `mdp_bringup`: launch, URDF, RViz
- [ ] End-to-end and fault tests, sanitizer + clang-tidy CI jobs
- [ ] Benchmarks vs the original Python planner
- [ ] Stretch: Gazebo + LiDAR + `slam_toolbox`, Nav2 planner plugin

## Build (once packages land)

```bash
# Ubuntu 22.04, ROS 2 Humble
mkdir -p ~/mdp_ws && cd ~/mdp_ws
git clone https://github.com/suujithhhh/mdp-robot-autonomy-stack.git src/mdp-robot-autonomy-stack
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

## Tech

C++17 · ROS 2 Humble · CMake / colcon · GTest · launch_testing · GitHub Actions · clang-tidy · ASan / UBSan · RViz

## License

MIT
