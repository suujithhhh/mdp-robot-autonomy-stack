// The six STM motion primitives and their exact constant-radius arc geometry.
#pragma once

#include <array>
#include <string>

#include "mdp_planner_core/types.hpp"

namespace mdp {

enum class Cmd : int { F = 0, B = 1, FL = 2, FR = 3, BL = 4, BR = 5 };

constexpr int kNumCmds = 6;

const char* cmd_name(Cmd c);
bool is_straight(Cmd c);
bool is_turn(Cmd c);
bool is_reverse(Cmd c);
// Largest legal STM value: 999 cm for F/B, 360 deg for turns. Smallest is 1.
int max_value(Cmd c);
bool value_is_legal(Cmd c, int value);

// Wrap an angle to [-pi, pi) the way Python's (a + pi) % (2*pi) - pi does.
double normalize_heading(double angle);

double turn_radius_cm(Cmd c, const PlannerConfig& cfg);
// Fixed centre of curvature of a turn primitive started at `pose`.
Point turn_center(const Pose& pose, Cmd c, const PlannerConfig& cfg);
// Travel length: cm for straights, radius * radians(value) for turns.
double primitive_length_cm(Cmd c, int value, const PlannerConfig& cfg);
// Pose after executing the whole primitive.
Pose propagate(const Pose& pose, Cmd c, int value, const PlannerConfig& cfg);
// Pose part-way through a primitive, fraction in [0, 1]. fraction = 1 equals propagate().
// `value` may be fractional here only because it is "this much of the integer command".
Pose pose_at_fraction(const Pose& pose, Cmd c, int value, double fraction, const PlannerConfig& cfg);

}  // namespace mdp
