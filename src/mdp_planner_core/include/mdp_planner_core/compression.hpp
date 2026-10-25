// Merges adjacent same-command lattice actions (F10+F10+F10 -> F30) after re-validating
// the merged command's whole swept path. Never changes the physical path.
#pragma once

#include <vector>

#include "mdp_planner_core/lattice.hpp"

namespace mdp {

LatticePlan compress_lattice_plan(const LatticePlan& plan, const std::vector<Obstacle>& obstacles,
                                  const PlannerConfig& cfg);

// "F,30" / "FL,90" wire string of one command.
std::string wire_string(Cmd c, int value);

}  // namespace mdp
