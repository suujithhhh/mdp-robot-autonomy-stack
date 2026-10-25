#include "mdp_planner_core/primitives.hpp"

#include <cmath>

namespace mdp {

namespace {
// +1: steering lock that increases heading when driving forward (FL, BR); -1: FR, BL.
constexpr int kSteerSign[kNumCmds] = {0, 0, +1, -1, -1, +1};
// +1 forward travel (F, FL, FR), -1 backward (B, BL, BR).
constexpr int kDistanceSign[kNumCmds] = {+1, -1, +1, +1, -1, -1};
}  // namespace

const char* cmd_name(Cmd c) {
  static const char* names[kNumCmds] = {"F", "B", "FL", "FR", "BL", "BR"};
  return names[static_cast<int>(c)];
}
bool is_straight(Cmd c) { return c == Cmd::F || c == Cmd::B; }
bool is_turn(Cmd c) { return !is_straight(c); }
bool is_reverse(Cmd c) { return c == Cmd::B || c == Cmd::BL || c == Cmd::BR; }
int max_value(Cmd c) { return is_straight(c) ? 999 : 360; }
bool value_is_legal(Cmd c, int value) { return value >= 1 && value <= max_value(c); }

double normalize_heading(double angle) {
  const double m = 2 * kPi;
  double r = std::fmod(angle + kPi, m);
  // Python's % takes the sign of the divisor, C's fmod takes the sign of the dividend.
  if (r != 0.0 && r < 0.0) r += m;
  return r - kPi;
}

double turn_radius_cm(Cmd c, const PlannerConfig& cfg) {
  switch (c) {
    case Cmd::FL:
      return cfg.turn_radius_fl_cm;
    case Cmd::FR:
      return cfg.turn_radius_fr_cm;
    case Cmd::BL:
      return cfg.turn_radius_bl_cm;
    case Cmd::BR:
      return cfg.turn_radius_br_cm;
    default:
      return 0.0;
  }
}

Point turn_center(const Pose& pose, Cmd c, const PlannerConfig& cfg) {
  const int steer = kSteerSign[static_cast<int>(c)];
  const double r = turn_radius_cm(c, cfg);
  return {pose.x + steer * r * (-std::sin(pose.heading)),
          pose.y + steer * r * std::cos(pose.heading)};
}

double primitive_length_cm(Cmd c, int value, const PlannerConfig& cfg) {
  if (is_straight(c)) return static_cast<double>(value);
  return turn_radius_cm(c, cfg) * (value * kDegToRad);
}

Pose pose_at_fraction(const Pose& pose, Cmd c, int value, double fraction,
                      const PlannerConfig& cfg) {
  if (fraction == 0.0) return pose;
  if (is_straight(c)) {
    const double d = kDistanceSign[static_cast<int>(c)] * static_cast<double>(value) * fraction;
    return {pose.x + d * std::cos(pose.heading), pose.y + d * std::sin(pose.heading), pose.heading};
  }
  const double theta = (value * kDegToRad) * fraction;
  const double delta = kSteerSign[static_cast<int>(c)] * kDistanceSign[static_cast<int>(c)] * theta;
  const Point ctr = turn_center(pose, c, cfg);
  const double rel_x = pose.x - ctr.x, rel_y = pose.y - ctr.y;
  const double cd = std::cos(delta), sd = std::sin(delta);
  return {ctr.x + (rel_x * cd - rel_y * sd), ctr.y + (rel_x * sd + rel_y * cd),
          normalize_heading(pose.heading + delta)};
}

Pose propagate(const Pose& pose, Cmd c, int value, const PlannerConfig& cfg) {
  return pose_at_fraction(pose, c, value, 1.0, cfg);
}

}  // namespace mdp
