#include "mdp_planner_core/swept.hpp"

#include <algorithm>
#include <cmath>

namespace mdp {

namespace {

constexpr int kSteerSign[kNumCmds] = {0, 0, +1, -1, -1, +1};
constexpr int kDistanceSign[kNumCmds] = {+1, -1, +1, +1, -1, -1};

struct SamplingPlan {
  int interval_count;
  double guard_cm;
};

// How many samples a primitive needs, and how much padding closes the gaps between them.
SamplingPlan plan_sampling(const Pose& start, Cmd c, int value, const PlannerConfig& cfg) {
  const double step = cfg.sweep_max_point_step_cm;
  if (is_straight(c)) {
    const double distance = std::fabs(static_cast<double>(value));
    const int n = std::max(1, static_cast<int>(std::ceil(distance / step)));
    return {n, (distance / n) / 2.0};
  }
  const double theta = value * kDegToRad;
  const Point ctr = turn_center(start, c, cfg);
  double max_radius = 0.0;  // farthest inflated-footprint corner from the turn centre
  for (const auto& p : footprint_corners(start, local_bounds(cfg))) {
    max_radius = std::max(max_radius, std::hypot(p.x - ctr.x, p.y - ctr.y));
  }
  const int n = std::max(1, static_cast<int>(std::ceil(max_radius * theta / step)));
  return {n, max_radius * ((theta / n) / 2.0)};
}

// i-th of n+1 uniformly spaced poses along the primitive (fraction = i / n).
// Per-primitive constants are computed once in the constructor.
class Sampler {
 public:
  Sampler(const Pose& start, Cmd c, int value, int n, const PlannerConfig& cfg)
      : start_(start), cmd_(c), n_(n) {
    if (is_straight(c)) {
      signed_value_ = kDistanceSign[static_cast<int>(c)] * static_cast<double>(value);
      cos_h_ = std::cos(start.heading);
      sin_h_ = std::sin(start.heading);
    } else {
      radians_value_ = value * kDegToRad;
      const Point ctr = turn_center(start, c, cfg);
      cx_ = ctr.x;
      cy_ = ctr.y;
      rel_x_ = start.x - cx_;
      rel_y_ = start.y - cy_;
      sign_ = kSteerSign[static_cast<int>(c)] * kDistanceSign[static_cast<int>(c)];
    }
  }
  Pose at(int i) const {
    if (i == 0) return start_;
    const double fraction = static_cast<double>(i) / n_;
    if (is_straight(cmd_)) {
      const double d = signed_value_ * fraction;
      return {start_.x + d * cos_h_, start_.y + d * sin_h_, start_.heading};
    }
    const double delta = sign_ * (radians_value_ * fraction);
    const double cd = std::cos(delta), sd = std::sin(delta);
    return {cx_ + (rel_x_ * cd - rel_y_ * sd), cy_ + (rel_x_ * sd + rel_y_ * cd),
            normalize_heading(start_.heading + delta)};
  }

 private:
  Pose start_;
  Cmd cmd_;
  int n_;
  double signed_value_{0}, cos_h_{0}, sin_h_{0};
  double radians_value_{0}, cx_{0}, cy_{0}, rel_x_{0}, rel_y_{0};
  int sign_{0};
};

}  // namespace

Samples sample_primitive(const Pose& start, Cmd c, int value, const PlannerConfig& cfg) {
  const SamplingPlan plan = plan_sampling(start, c, value, cfg);
  const Sampler sampler(start, c, value, plan.interval_count, cfg);
  Samples out;
  out.guard_cm = plan.guard_cm;
  out.poses.reserve(plan.interval_count + 1);
  for (int i = 0; i <= plan.interval_count; ++i) out.poses.push_back(sampler.at(i));
  return out;
}

std::optional<std::vector<Pose>> safe_primitive_samples(const Pose& start, Cmd c, int value,
                                                        const std::vector<Rect>& rects,
                                                        const PlannerConfig& cfg) {
  const SamplingPlan plan = plan_sampling(start, c, value, cfg);
  const Sampler sampler(start, c, value, plan.interval_count, cfg);
  const LocalBounds bounds = local_bounds(cfg, plan.guard_cm);
  std::vector<Pose> poses;
  poses.reserve(plan.interval_count + 1);
  for (int i = 0; i <= plan.interval_count; ++i) {
    poses.push_back(sampler.at(i));
    if (!pose_is_safe(poses.back(), bounds, rects, cfg.arena_cm)) return std::nullopt;
  }
  return poses;
}

std::optional<Pose> safe_primitive_end(const Pose& start, Cmd c, int value,
                                       const std::vector<Rect>& rects, const PlannerConfig& cfg) {
  const SamplingPlan plan = plan_sampling(start, c, value, cfg);
  const Sampler sampler(start, c, value, plan.interval_count, cfg);
  const LocalBounds bounds = local_bounds(cfg, plan.guard_cm);
  Pose last = start;
  for (int i = 0; i <= plan.interval_count; ++i) {
    last = sampler.at(i);
    if (!pose_is_safe(last, bounds, rects, cfg.arena_cm)) return std::nullopt;
  }
  return last;
}

}  // namespace mdp
