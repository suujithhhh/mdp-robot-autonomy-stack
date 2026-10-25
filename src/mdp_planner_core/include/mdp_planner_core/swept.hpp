// Swept-path collision checking: samples a primitive's WHOLE path (not just its endpoint)
// and checks the margin-inflated footprint at every sample, padded by an anti-tunnelling guard.
#pragma once

#include <optional>
#include <vector>

#include "mdp_planner_core/footprint.hpp"
#include "mdp_planner_core/primitives.hpp"

namespace mdp {

struct Samples {
  std::vector<Pose> poses;  // includes the exact start pose and the exact end pose
  double guard_cm{0.0};
};

// Uniformly spaced poses along one primitive, spaced so no footprint point moves more
// than cfg.sweep_max_point_step_cm between neighbours.
Samples sample_primitive(const Pose& start, Cmd c, int value, const PlannerConfig& cfg);

// Obstacle rectangles are built once per leg and passed in; returns the samples when the
// whole swept path is safe, nullopt otherwise.
std::optional<std::vector<Pose>> safe_primitive_samples(const Pose& start, Cmd c, int value,
                                                        const std::vector<Rect>& rects,
                                                        const PlannerConfig& cfg);

// Cheaper variant for the search hot loop: only the final pose is returned.
std::optional<Pose> safe_primitive_end(const Pose& start, Cmd c, int value,
                                       const std::vector<Rect>& rects, const PlannerConfig& cfg);

}  // namespace mdp
