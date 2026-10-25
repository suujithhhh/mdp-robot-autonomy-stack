// Mission executor: sends a planned command list to the robot one command at a time and reacts
// to failures the way PROTOCOL.md section 5 prescribes. Independent of ROS and of the transport:
// it talks to a Link, which is the in-memory simulator in tests and the serial port in the node.
#pragma once

#include <string>
#include <vector>

#include "mdp_planner_core/route.hpp"
#include "stm32_bridge/bridge_core.hpp"

namespace mission {

// What the executor needs from a transport: the driver, a clock, and a way to let time pass
// (pump bytes in both directions) for dt seconds.
class Link {
 public:
  virtual ~Link() = default;
  virtual stm::BridgeCore& bridge() = 0;
  virtual double now_s() const = 0;
  virtual void advance(double dt_s) = 0;
};

struct Snapshot {
  int obstacle_id;
  mdp::Pose target;      // the viewing pose the planner aimed at
  mdp::Pose commanded;   // dead-reckoned estimate (ACK-based) when the leg finished
  mdp::Pose odometry;    // telemetry-integrated estimate
};

struct MissionOptions {
  int max_retries{1};   // only for ERR,IMU_NOT_READY on a command that never started moving
  double step_s{0.01};
  double max_wait_s{600.0};  // hard cap so a test can never hang
};

struct MissionResult {
  bool completed{false};
  std::size_t commands_done{0};
  std::size_t commands_total{0};
  stm::Outcome failure{stm::Outcome::Ack};   // meaningful when !completed
  std::string failed_wire;
  bool pose_uncertain{false};
  int retries{0};
  double elapsed_s{0.0};
  std::vector<Snapshot> snapshots;
};

// Flatten a route into the command list and the index at which each leg ends.
struct FlatRoute {
  std::vector<stm::MoveCmd> commands;
  std::vector<std::size_t> leg_end_index;  // commands.size() after each leg
};
FlatRoute flatten(const mdp::RouteResult& route);

// Run the route. `route` supplies the snapshot targets; `commands` may be a prefix of it.
MissionResult run_mission(const mdp::RouteResult& route, Link& link, const MissionOptions& opts = {});

}  // namespace mission
