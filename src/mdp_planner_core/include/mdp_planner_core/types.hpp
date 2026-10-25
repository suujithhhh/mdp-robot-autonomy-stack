// Basic value types and the planner configuration.
// Defaults mirror algo/config.py of the MDP Python planner (PlannerConfig).
#pragma once

#include <cmath>
#include <string>

namespace mdp {

struct Point {
  double x, y;
};

// Rear-axle-midpoint pose. heading in radians, 0 = +x, counter-clockwise.
struct Pose {
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  bool operator==(const Pose& o) const { return x == o.x && y == o.y && heading == o.heading; }
};

// 10x10 cm obstacle whose lower-left corner is (x, y); face in {N,E,S,W} is the image face.
struct Obstacle {
  int id{0};
  double x{0.0};
  double y{0.0};
  char face{'N'};
};

struct PlannerConfig {
  double arena_cm = 200.0;
  double obstacle_cm = 10.0;
  // Footprint about the rear reference point.
  double robot_front_extent_cm = 24.0;
  double robot_rear_extent_cm = 0.0;
  double robot_width_cm = 19.0;
  double safety_margin_cm = 3.0;
  // Camera.
  double camera_forward_offset_cm = 15.25;
  double camera_lateral_offset_cm = 0.0;
  double camera_standoff_cm = 25.0;
  // Four independently measured turn radii (never mirror one from another).
  double turn_radius_fl_cm = 24.8;
  double turn_radius_fr_cm = 33.7;
  double turn_radius_bl_cm = 33.6;
  double turn_radius_br_cm = 24.5;
  // Costs.
  double reverse_penalty = 1.08;
  double straight_command_cost_cm = 66.0;
  double turn_command_cost_cm = 34.0;
  // Collision sampling.
  double sweep_max_point_step_cm = 1.0;
  // Lattice search.
  int lattice_straight_step_cm = 10;
  int lattice_turn_step_deg = 10;
  double lattice_position_bin_cm = 5.0;
  int lattice_heading_bin_deg = 10;
  double lattice_goal_position_tolerance_cm = 3.0;
  double lattice_goal_heading_tolerance_deg = 5.0;
  int lattice_max_expansions = 200000;
};

// Same constants CPython uses for math.radians / math.degrees, so results match bit for bit.
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

}  // namespace mdp
