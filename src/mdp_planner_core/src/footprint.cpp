#include "mdp_planner_core/footprint.hpp"

#include <algorithm>

namespace mdp {

LocalBounds local_bounds(const PlannerConfig& cfg, double guard_cm) {
  const double margin = cfg.safety_margin_cm + guard_cm;
  const double half_width = cfg.robot_width_cm / 2;
  return {-cfg.robot_rear_extent_cm - margin, cfg.robot_front_extent_cm + margin,
          -half_width - margin, half_width + margin};
}

std::array<Point, 4> footprint_corners(const Pose& pose, const LocalBounds& b) {
  const double c = std::cos(pose.heading), s = std::sin(pose.heading);
  // rear-right, front-right, front-left, rear-left (same order as the Python reference)
  return {{{pose.x + b.rear * c - b.right * s, pose.y + b.rear * s + b.right * c},
           {pose.x + b.front * c - b.right * s, pose.y + b.front * s + b.right * c},
           {pose.x + b.front * c - b.left * s, pose.y + b.front * s + b.left * c},
           {pose.x + b.rear * c - b.left * s, pose.y + b.rear * s + b.left * c}}};
}

std::vector<Rect> make_rects(const std::vector<Obstacle>& obstacles, const PlannerConfig& cfg) {
  std::vector<Rect> rects;
  rects.reserve(obstacles.size());
  for (const auto& o : obstacles) {
    rects.push_back({o.x, o.x + cfg.obstacle_cm, o.y, o.y + cfg.obstacle_cm});
  }
  return rects;
}

namespace {
// Do the projections of the footprint (p) and the obstacle (q) overlap on one axis?
bool overlap(double pmax, double pmin, double qmax, double qmin) {
  return pmax > qmin && qmax > pmin;
}
}  // namespace

bool pose_is_safe(const Pose& pose, const LocalBounds& b, const std::vector<Rect>& rects,
                  double arena_cm) {
  const auto pts = footprint_corners(pose, b);
  const double x0 = pts[0].x, y0 = pts[0].y, x1 = pts[1].x, y1 = pts[1].y;
  const double x2 = pts[2].x, y2 = pts[2].y, x3 = pts[3].x, y3 = pts[3].y;

  // Inside the arena (touching the wall is allowed).
  for (const auto& p : pts) {
    if (!(0 <= p.x && p.x <= arena_cm && 0 <= p.y && p.y <= arena_cm)) return false;
  }
  if (rects.empty()) return true;

  const double c = std::cos(pose.heading), s = std::sin(pose.heading);
  const double min_x = std::min({x0, x1, x2, x3}), max_x = std::max({x0, x1, x2, x3});
  const double min_y = std::min({y0, y1, y2, y3}), max_y = std::max({y0, y1, y2, y3});
  const double ns = -s;
  for (const auto& r : rects) {
    // Axes (1,0) and (0,1): axis-aligned bounding boxes.
    if (!(max_x > r.left && r.right > min_x && max_y > r.bottom && r.top > min_y)) continue;
    // Axis (cos, sin): the robot's long axis.
    double p0 = x0 * c + y0 * s, p1 = x1 * c + y1 * s, p2 = x2 * c + y2 * s, p3 = x3 * c + y3 * s;
    double q0 = r.left * c + r.bottom * s, q1 = r.right * c + r.bottom * s;
    double q2 = r.right * c + r.top * s, q3 = r.left * c + r.top * s;
    if (!overlap(std::max({p0, p1, p2, p3}), std::min({p0, p1, p2, p3}),
                 std::max({q0, q1, q2, q3}), std::min({q0, q1, q2, q3})))
      continue;
    // Axis (-sin, cos): the robot's short axis.
    p0 = x0 * ns + y0 * c;
    p1 = x1 * ns + y1 * c;
    p2 = x2 * ns + y2 * c;
    p3 = x3 * ns + y3 * c;
    q0 = r.left * ns + r.bottom * c;
    q1 = r.right * ns + r.bottom * c;
    q2 = r.right * ns + r.top * c;
    q3 = r.left * ns + r.top * c;
    if (overlap(std::max({p0, p1, p2, p3}), std::min({p0, p1, p2, p3}),
                std::max({q0, q1, q2, q3}), std::min({q0, q1, q2, q3})))
      return false;  // overlaps on all four axes -> collision
  }
  return true;
}

bool pose_is_safe(const Pose& pose, const std::vector<Obstacle>& obstacles,
                  const PlannerConfig& cfg) {
  return pose_is_safe(pose, local_bounds(cfg), make_rects(obstacles, cfg), cfg.arena_cm);
}

bool footprint_in_arena(const Pose& pose, const PlannerConfig& cfg) {
  return pose_is_safe(pose, local_bounds(cfg), {}, cfg.arena_cm);
}

}  // namespace mdp
