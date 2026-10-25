#include "mdp_planner_core/compression.hpp"

#include <cmath>
#include <stdexcept>

#include "mdp_planner_core/footprint.hpp"
#include "mdp_planner_core/swept.hpp"

namespace mdp {

namespace {

// Round-off-only tolerances: merged and unmerged replays differ only by floating-point noise.
constexpr double kStrictPositionTolCm = 1e-6;
constexpr double kStrictHeadingTolRad = 1e-9;

bool poses_match_strict(const Pose& a, const Pose& b) {
  if (std::hypot(a.x - b.x, a.y - b.y) > kStrictPositionTolCm) return false;
  return std::fabs(normalize_heading(a.heading - b.heading)) <= kStrictHeadingTolRad;
}

// A run of adjacent actions that share one command.
using Chunk = std::vector<Action>;

// Split into maximal runs of the same command, then into runs whose summed value stays a legal
// STM value. An original action is never split.
std::vector<Chunk> make_chunks(const std::vector<Action>& actions) {
  std::vector<Chunk> chunks;
  int running_sum = 0;
  for (const auto& a : actions) {
    const bool same = !chunks.empty() && chunks.back().back().cmd == a.cmd;
    if (same && value_is_legal(a.cmd, running_sum + a.value)) {
      chunks.back().push_back(a);
      running_sum += a.value;
    } else {
      chunks.push_back({a});
      running_sum = a.value;
    }
  }
  return chunks;
}

}  // namespace

std::string wire_string(Cmd c, int value) {
  return std::string(cmd_name(c)) + "," + std::to_string(value);
}

LatticePlan compress_lattice_plan(const LatticePlan& plan, const std::vector<Obstacle>& obstacles,
                                  const PlannerConfig& cfg) {
  if (!plan.valid || plan.actions.empty()) return plan;

  const std::vector<Rect> rects = make_rects(obstacles, cfg);
  std::vector<Action> out;
  Pose current = plan.actions.front().start;

  for (const auto& chunk : make_chunks(plan.actions)) {
    bool merged = false;
    if (chunk.size() > 1) {
      const Cmd cmd = chunk.front().cmd;
      int combined = 0;
      for (const auto& a : chunk) combined += a.value;
      // Re-validate the merged command's whole swept path: never assume it is safe because its
      // parts were.
      const auto samples = safe_primitive_samples(current, cmd, combined, rects, cfg);
      if (samples) {
        Pose replay = current;
        for (const auto& a : chunk) replay = propagate(replay, a.cmd, a.value, cfg);
        if (poses_match_strict(samples->back(), replay)) {
          out.push_back({cmd, combined, current, samples->back(),
                         primitive_length_cm(cmd, combined, cfg)});
          current = samples->back();
          merged = true;
        }
      }
    }
    if (merged) continue;
    // Fallback: replay each original action, re-validated from the exact current pose.
    for (const auto& a : chunk) {
      const auto samples = safe_primitive_samples(current, a.cmd, a.value, rects, cfg);
      if (!samples) {
        throw std::runtime_error("compression: original action failed re-validation");
      }
      out.push_back({a.cmd, a.value, current, samples->back(),
                     primitive_length_cm(a.cmd, a.value, cfg)});
      current = samples->back();
    }
  }

  if (!poses_match_strict(current, plan.final_pose)) {
    throw std::runtime_error("compression: final pose disagrees with the input plan");
  }

  LatticePlan result;
  result.valid = true;
  result.actions = std::move(out);
  for (const auto& a : result.actions) result.total_distance_cm += a.length_cm;
  result.total_cost = path_cost_cm(result.actions, cfg);
  result.expansions = plan.expansions;
  result.final_pose = current;
  if (std::fabs(result.total_cost - plan.total_cost) > 1e-6) {
    throw std::runtime_error("compression: total cost changed");
  }
  return result;
}

}  // namespace mdp
