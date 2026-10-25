// Multi-obstacle route planning: choose the visiting order with branch-and-bound pruned by a
// Held-Karp lower bound, planning each leg with the lattice search and a hard deadline.
#pragma once

#include <chrono>
#include <functional>
#include <optional>
#include <vector>

#include "mdp_planner_core/compression.hpp"

namespace mdp {

// Rear-reference pose that puts the camera lens `camera_standoff_cm` from the image face.
Pose viewing_target(const Obstacle& obstacle, const PlannerConfig& cfg);

// One leg = lattice search + command compression. Pluggable so tests can inject a fake.
using LegPlanner = std::function<LatticePlan(const Pose&, const Pose&, Clock::time_point)>;
LegPlanner default_leg_planner(const std::vector<Obstacle>& obstacles, const PlannerConfig& cfg);

struct RouteOptions {
  // Wall-clock budget for the search. When it runs out, the best VERIFIED route found so far
  // is returned and `timed_out` is set.
  std::chrono::milliseconds time_limit{70000};
  bool use_held_karp_bound{true};
};

struct RouteLeg {
  int obstacle_id;
  Pose target;
  LatticePlan plan;  // compressed
};

struct RouteStats {
  long leg_requests{0};
  long leg_plans{0};      // cache misses = legs actually planned
  long bound_prunes{0};
  long incumbent_updates{0};
  long nodes_visited{0};
  long failed_leg_plans{0};      // legs that returned invalid (unreachable / expansion limit)
  double leg_time_ms{0.0};       // total wall time spent inside the leg planner
  double failed_leg_time_ms{0.0};
};

struct RouteResult {
  bool valid{false};     // at least one leg planned
  bool complete{false};  // every obstacle visited
  bool timed_out{false};
  std::vector<RouteLeg> legs;
  double total_cost{0.0};
  RouteStats stats;
  std::string reason;
};

// Admissible lower bound on the cost of visiting `remaining` (bit mask) from `current`,
// exposed for testing. Built from pairwise distance bounds with a Held-Karp DP.
class HeldKarpBound {
 public:
  HeldKarpBound(const std::vector<Pose>& targets, const PlannerConfig& cfg);
  // Lower bound for visiting every target in `remaining`, starting from actual pose `from`.
  double lower_bound(const Pose& from, unsigned remaining) const;

 private:
  double edge(int a, int b) const;
  std::vector<Pose> targets_;
  PlannerConfig cfg_;
  // path_[j][S] = cheapest lower-bound path starting at target j that visits every target in S.
  std::vector<std::vector<double>> path_;
};

RouteResult optimise_route(const Pose& start, const std::vector<Obstacle>& obstacles,
                           const PlannerConfig& cfg, const LegPlanner& planner,
                           const RouteOptions& opts = {});

}  // namespace mdp
