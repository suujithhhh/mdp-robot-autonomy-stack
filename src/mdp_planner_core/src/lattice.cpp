#include "mdp_planner_core/lattice.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

#include "mdp_planner_core/footprint.hpp"
#include "mdp_planner_core/swept.hpp"

namespace mdp {

const char* reason_name(FailReason r) {
  switch (r) {
    case FailReason::None:
      return "";
    case FailReason::UnsafeStart:
      return "unsafe_start";
    case FailReason::UnsafeGoal:
      return "unsafe_goal";
    case FailReason::ExpansionLimit:
      return "expansion_limit";
    case FailReason::NoPath:
      return "no_path";
    case FailReason::Timeout:
      return "timeout";
  }
  return "";
}

double command_start_cost_cm(Cmd c, int previous_cmd, const PlannerConfig& cfg) {
  if (static_cast<int>(c) == previous_cmd) return 0.0;  // continues the previous command
  return is_straight(c) ? cfg.straight_command_cost_cm : cfg.turn_command_cost_cm;
}

double edge_cost_cm(Cmd c, double length_cm, const PlannerConfig& cfg) {
  return is_reverse(c) ? length_cm * cfg.reverse_penalty : length_cm;
}

double path_cost_cm(const std::vector<Action>& actions, const PlannerConfig& cfg) {
  double total = 0.0;
  int previous = -1;
  for (const auto& a : actions) {
    total += edge_cost_cm(a.cmd, a.length_cm, cfg) + command_start_cost_cm(a.cmd, previous, cfg);
    previous = static_cast<int>(a.cmd);
  }
  return total;
}

namespace {

constexpr int kStartAction = kNumCmds;  // "previous action" of the initial state

struct Primitive {
  Cmd cmd;
  int value;
};

// Fixed, stable expansion order: F, B, FL, FR, BL, BR.
std::array<Primitive, kNumCmds> successor_primitives(const PlannerConfig& cfg) {
  const int s = cfg.lattice_straight_step_cm, t = cfg.lattice_turn_step_deg;
  return {{{Cmd::F, s}, {Cmd::B, s}, {Cmd::FL, t}, {Cmd::FR, t}, {Cmd::BL, t}, {Cmd::BR, t}}};
}

// Python's float %: the result takes the sign of the divisor.
double py_mod(double a, double m) {
  double r = std::fmod(a, m);
  if (r != 0.0 && (r < 0.0) != (m < 0.0)) r += m;
  return r;
}

int position_bin(double value_cm, double bin_cm) {
  return static_cast<int>(std::nearbyint(value_cm / bin_cm));  // ties to even, like round()
}

int heading_bin(double heading_rad, int bin_deg) {
  const double degrees = py_mod(normalize_heading(heading_rad) * kRadToDeg, 360.0);
  const int bin_count = 360 / bin_deg;
  const int b = static_cast<int>(std::nearbyint(degrees / bin_deg));
  return ((b % bin_count) + bin_count) % bin_count;
}

// Duplicate-detection key: quantised position, quantised heading, and the previous command
// (the per-command cost depends on it).
std::uint64_t state_key(const Pose& p, int previous_action, const PlannerConfig& cfg) {
  const std::uint64_t bx =
      static_cast<std::uint64_t>(position_bin(p.x, cfg.lattice_position_bin_cm) + 32768);
  const std::uint64_t by =
      static_cast<std::uint64_t>(position_bin(p.y, cfg.lattice_position_bin_cm) + 32768);
  const std::uint64_t hb =
      static_cast<std::uint64_t>(heading_bin(p.heading, cfg.lattice_heading_bin_deg));
  return (bx << 40) | (by << 24) | (hb << 8) | static_cast<std::uint64_t>(previous_action);
}

bool within_goal_tolerance(const Pose& p, const Pose& goal, const PlannerConfig& cfg) {
  if (std::hypot(p.x - goal.x, p.y - goal.y) > cfg.lattice_goal_position_tolerance_cm) return false;
  double diff = std::fabs(normalize_heading(p.heading - goal.heading) * kRadToDeg);
  diff = std::min(diff, 360.0 - diff);
  return diff <= cfg.lattice_goal_heading_tolerance_deg;
}

// Admissible lower bound on the remaining cost: max(distance bound, rotation bound) plus the
// fixed cost of one more turn command if the heading is still outside tolerance.
double search_heuristic(const Pose& p, const Pose& goal, int previous_action,
                        const PlannerConfig& cfg) {
  const double position_distance = std::hypot(p.x - goal.x, p.y - goal.y);
  const double distance_part =
      std::max(0.0, position_distance - cfg.lattice_goal_position_tolerance_cm) *
      std::min(1.0, cfg.reverse_penalty);
  const double tolerance = cfg.lattice_goal_heading_tolerance_deg * kDegToRad;
  const double ccw = py_mod(goal.heading - p.heading, 2.0 * kPi);
  const double ccw_need = std::max(0.0, ccw - tolerance);
  const double cw_need = std::max(0.0, 2.0 * kPi - ccw - tolerance);
  if (ccw_need == 0.0 || cw_need == 0.0) return distance_part;
  const double rp = cfg.reverse_penalty;
  const double ccw_rate = std::min(cfg.turn_radius_fl_cm, cfg.turn_radius_bl_cm * rp);
  const double cw_rate = std::min(cfg.turn_radius_fr_cm, cfg.turn_radius_br_cm * rp);
  const double heading_part = std::min(ccw_rate * ccw_need, cw_rate * cw_need);
  const bool prev_is_turn = previous_action >= 0 && previous_action < kNumCmds &&
                            is_turn(static_cast<Cmd>(previous_action));
  const double command_part = prev_is_turn ? 0.0 : cfg.turn_command_cost_cm;
  return std::max(distance_part, heading_part) + command_part;
}

struct Node {
  Pose pose;
  double g;
  int parent;  // index into the node pool, -1 for the start
  int action;  // Cmd as int, kStartAction for the start
  int value;
  double length_cm;
  std::uint64_t key;
};

struct HeapEntry {
  double f;
  std::uint64_t order;  // insertion counter: ties break first-in-first-out
  int node;
  bool operator>(const HeapEntry& o) const { return f != o.f ? f > o.f : order > o.order; }
};

LatticePlan failure(FailReason reason, long expansions) {
  LatticePlan p;
  p.reason = reason;
  p.expansions = expansions;
  return p;
}

}  // namespace

LatticePlan plan_lattice_path(const Pose& start, const Pose& goal,
                              const std::vector<Obstacle>& obstacles, const PlannerConfig& cfg,
                              Clock::time_point deadline) {
  const std::vector<Rect> rects = make_rects(obstacles, cfg);
  const LocalBounds bounds = local_bounds(cfg);
  if (!pose_is_safe(start, bounds, rects, cfg.arena_cm)) return failure(FailReason::UnsafeStart, 0);
  if (!pose_is_safe(goal, bounds, rects, cfg.arena_cm)) return failure(FailReason::UnsafeGoal, 0);

  if (within_goal_tolerance(start, goal, cfg)) {
    LatticePlan p;
    p.valid = true;
    p.final_pose = start;
    return p;
  }

  const auto primitives = successor_primitives(cfg);
  std::vector<Node> pool;
  pool.reserve(4096);
  std::unordered_map<std::uint64_t, double> best_g;
  std::priority_queue<HeapEntry, std::vector<HeapEntry>, std::greater<HeapEntry>> open;
  std::uint64_t counter = 0;

  const std::uint64_t start_key = state_key(start, kStartAction, cfg);
  pool.push_back({start, 0.0, -1, kStartAction, 0, 0.0, start_key});
  best_g[start_key] = 0.0;
  open.push({search_heuristic(start, goal, -1, cfg), counter++, 0});

  long expansions = 0;
  const bool has_deadline = deadline != Clock::time_point::max();
  while (!open.empty()) {
    if (expansions >= cfg.lattice_max_expansions)
      return failure(FailReason::ExpansionLimit, expansions);
    if (has_deadline && (expansions & 255) == 0 && Clock::now() >= deadline)
      return failure(FailReason::Timeout, expansions);

    const HeapEntry top = open.top();
    open.pop();
    const int idx = top.node;
    const Node node = pool[idx];  // copy: pool may grow below
    const auto found = best_g.find(node.key);
    if (found != best_g.end() && node.g > found->second) continue;  // stale entry

    ++expansions;

    if (within_goal_tolerance(node.pose, goal, cfg)) {
      LatticePlan plan;
      plan.valid = true;
      plan.expansions = expansions;
      plan.total_cost = node.g;
      plan.final_pose = node.pose;
      for (int i = idx; pool[i].parent != -1; i = pool[i].parent) {
        const Node& n = pool[i];
        plan.actions.push_back(
            {static_cast<Cmd>(n.action), n.value, pool[n.parent].pose, n.pose, n.length_cm});
      }
      std::reverse(plan.actions.begin(), plan.actions.end());
      for (const auto& a : plan.actions) plan.total_distance_cm += a.length_cm;
      return plan;
    }

    for (const auto& prim : primitives) {
      const auto end_pose = safe_primitive_end(node.pose, prim.cmd, prim.value, rects, cfg);
      if (!end_pose) continue;
      const double length = primitive_length_cm(prim.cmd, prim.value, cfg);
      const double edge =
          edge_cost_cm(prim.cmd, length, cfg) + command_start_cost_cm(prim.cmd, node.action, cfg);
      const double new_g = node.g + edge;
      const int action = static_cast<int>(prim.cmd);
      const std::uint64_t key = state_key(*end_pose, action, cfg);
      const auto it = best_g.find(key);
      if (it != best_g.end() && new_g >= it->second) continue;
      best_g[key] = new_g;
      pool.push_back({*end_pose, new_g, idx, action, prim.value, length, key});
      open.push({new_g + search_heuristic(*end_pose, goal, action, cfg), counter++,
                 static_cast<int>(pool.size()) - 1});
    }
  }
  return failure(FailReason::NoPath, expansions);
}

}  // namespace mdp
