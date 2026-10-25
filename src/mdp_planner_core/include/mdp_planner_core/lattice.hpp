// State-lattice A* search for one collision-free leg using only the six STM primitives.
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "mdp_planner_core/primitives.hpp"
#include "mdp_planner_core/types.hpp"

namespace mdp {

struct Action {
  Cmd cmd;
  int value;
  Pose start;
  Pose end;
  double length_cm;
};

enum class FailReason { None, UnsafeStart, UnsafeGoal, ExpansionLimit, NoPath, Timeout };

using Clock = std::chrono::steady_clock;
const char* reason_name(FailReason r);

struct LatticePlan {
  bool valid{false};
  std::vector<Action> actions;
  double total_distance_cm{0.0};
  double total_cost{0.0};
  long expansions{0};
  Pose final_pose{};
  FailReason reason{FailReason::None};
};

// Cost of a leg: sum of step costs, plus a fixed command cost for each step whose command
// differs from the previous step (compression merges runs of equal steps into one command).
double command_start_cost_cm(Cmd c, int previous_cmd /* -1 = none */, const PlannerConfig& cfg);
double edge_cost_cm(Cmd c, double length_cm, const PlannerConfig& cfg);
double path_cost_cm(const std::vector<Action>& actions, const PlannerConfig& cfg);

// `deadline` bounds wall-clock time; the search returns FailReason::Timeout instead of hanging.
LatticePlan plan_lattice_path(const Pose& start, const Pose& goal,
                              const std::vector<Obstacle>& obstacles, const PlannerConfig& cfg,
                              Clock::time_point deadline = Clock::time_point::max());

}  // namespace mdp
