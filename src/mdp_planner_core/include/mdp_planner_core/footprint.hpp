// Robot footprint geometry and fast collision checking against 10x10 cm obstacles.
#pragma once

#include <array>
#include <vector>

#include "mdp_planner_core/types.hpp"

namespace mdp {

// Obstacle rectangle with precomputed bounds. Built once per leg, reused by every sample.
struct Rect {
  double left, right, bottom, top;
};

// Footprint bounds in the robot frame: rear/right are negative.
struct LocalBounds {
  double rear, front, right, left;
};

// `guard_cm` is extra padding on top of the physical safety margin (anti-tunnelling guard).
LocalBounds local_bounds(const PlannerConfig& cfg, double guard_cm = 0.0);

std::array<Point, 4> footprint_corners(const Pose& pose, const LocalBounds& b);

std::vector<Rect> make_rects(const std::vector<Obstacle>& obstacles, const PlannerConfig& cfg);

// True when the margin-inflated footprint is inside the arena (touching allowed) and
// does not overlap any obstacle. Separating-axis test on four axes.
bool pose_is_safe(const Pose& pose, const LocalBounds& b, const std::vector<Rect>& rects,
                  double arena_cm);

// Convenience: physical margin only, no guard.
bool pose_is_safe(const Pose& pose, const std::vector<Obstacle>& obstacles,
                  const PlannerConfig& cfg);

bool footprint_in_arena(const Pose& pose, const PlannerConfig& cfg);

}  // namespace mdp
