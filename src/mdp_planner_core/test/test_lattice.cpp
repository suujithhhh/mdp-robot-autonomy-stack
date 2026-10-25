#include <gtest/gtest.h>

#include "mdp_planner_core/compression.hpp"
#include "mdp_planner_core/lattice.hpp"
#include "mdp_planner_core/swept.hpp"

using namespace mdp;

namespace {
const PlannerConfig cfg;

// Replays a plan with propagate() and returns the reached pose.
Pose replay(const Pose& start, const LatticePlan& plan) {
  Pose p = start;
  for (const auto& a : plan.actions) p = propagate(p, a.cmd, a.value, cfg);
  return p;
}

bool near_goal(const Pose& p, const Pose& g) {
  return std::hypot(p.x - g.x, p.y - g.y) <= cfg.lattice_goal_position_tolerance_cm + 1e-6;
}
}  // namespace

TEST(Lattice, StartAlreadyAtGoalNeedsNoActions) {
  const Pose p{100, 100, 0};
  const LatticePlan plan = plan_lattice_path(p, p, {}, cfg);
  EXPECT_TRUE(plan.valid);
  EXPECT_TRUE(plan.actions.empty());
  EXPECT_EQ(plan.total_cost, 0.0);
}

TEST(Lattice, StraightLineGoalIsOneCompressedForwardCommand) {
  const LatticePlan raw = plan_lattice_path({30, 100, 0}, {100, 100, 0}, {}, cfg);
  ASSERT_TRUE(raw.valid);
  const LatticePlan plan = compress_lattice_plan(raw, {}, cfg);
  ASSERT_EQ(plan.actions.size(), 1u);
  EXPECT_EQ(plan.actions[0].cmd, Cmd::F);
  EXPECT_NEAR(plan.actions[0].value, 70, 3);  // within the 3 cm goal tolerance
}

TEST(Lattice, UnsafeStartAndGoalAreRejectedImmediately) {
  const std::vector<Obstacle> obs{{1, 90, 90, 'N'}};
  const LatticePlan bad_start = plan_lattice_path({95, 95, 0}, {150, 150, 0}, obs, cfg);
  EXPECT_FALSE(bad_start.valid);
  EXPECT_EQ(bad_start.reason, FailReason::UnsafeStart);
  const LatticePlan bad_goal = plan_lattice_path({30, 30, 0}, {95, 95, 0}, obs, cfg);
  EXPECT_FALSE(bad_goal.valid);
  EXPECT_EQ(bad_goal.reason, FailReason::UnsafeGoal);
}

TEST(Lattice, PlannedPathReachesGoalWithinToleranceWhenReplayed) {
  const Pose start{20, 10, kPi / 2}, goal{80, 90, 0};
  const LatticePlan raw = plan_lattice_path(start, goal, {}, cfg);
  ASSERT_TRUE(raw.valid);
  EXPECT_TRUE(near_goal(replay(start, raw), goal));
}

TEST(Lattice, EverySampleOfAPlannedPathIsCollisionFree) {
  const std::vector<Obstacle> obs{{1, 60, 60, 'S'}, {2, 110, 40, 'W'}};
  const Pose start{20, 20, 0}, goal{150, 120, kPi / 2};
  const LatticePlan raw = plan_lattice_path(start, goal, obs, cfg);
  ASSERT_TRUE(raw.valid);
  const auto rects = make_rects(obs, cfg);
  Pose p = start;
  for (const auto& a : raw.actions) {
    EXPECT_TRUE(safe_primitive_samples(p, a.cmd, a.value, rects, cfg).has_value())
        << cmd_name(a.cmd) << "," << a.value;
    p = propagate(p, a.cmd, a.value, cfg);
  }
}

TEST(Lattice, ExpansionLimitStopsTheSearchInsteadOfHanging) {
  PlannerConfig tiny = cfg;
  tiny.lattice_max_expansions = 5;
  const LatticePlan plan = plan_lattice_path({20, 20, 0}, {150, 120, kPi / 2}, {}, tiny);
  EXPECT_FALSE(plan.valid);
  EXPECT_EQ(plan.reason, FailReason::ExpansionLimit);
  EXPECT_EQ(plan.expansions, 5);
}

TEST(Lattice, DeadlineInThePastTimesOutInsteadOfSearching) {
  const LatticePlan plan = plan_lattice_path({20, 20, 0}, {150, 120, kPi / 2}, {}, cfg,
                                             Clock::now() - std::chrono::seconds(1));
  EXPECT_FALSE(plan.valid);
  EXPECT_EQ(plan.reason, FailReason::Timeout);
}

TEST(Lattice, CostChargesOneCommandStartPerRunOfEqualCommands) {
  std::vector<Action> a(3, Action{Cmd::F, 10, {}, {}, 10.0});
  EXPECT_DOUBLE_EQ(path_cost_cm(a, cfg), 66.0 + 30.0);
  a.push_back({Cmd::FL, 10, {}, {}, 4.0});
  EXPECT_DOUBLE_EQ(path_cost_cm(a, cfg), 66.0 + 30.0 + 34.0 + 4.0);
  a.push_back({Cmd::B, 10, {}, {}, 10.0});
  EXPECT_NEAR(path_cost_cm(a, cfg), 66.0 + 30.0 + 34.0 + 4.0 + 66.0 + 10.0 * 1.08, 1e-9);
}

TEST(Compression, MergesEqualRunsAndPreservesCostAndEndpoint) {
  const LatticePlan raw = plan_lattice_path({20, 10, kPi / 2}, {80, 90, 0}, {}, cfg);
  ASSERT_TRUE(raw.valid);
  const LatticePlan packed = compress_lattice_plan(raw, {}, cfg);
  EXPECT_LE(packed.actions.size(), raw.actions.size());
  EXPECT_NEAR(packed.total_cost, raw.total_cost, 1e-6);
  EXPECT_NEAR(packed.final_pose.x, raw.final_pose.x, 1e-6);
  EXPECT_NEAR(packed.final_pose.y, raw.final_pose.y, 1e-6);
  for (std::size_t i = 1; i < packed.actions.size(); ++i) {
    EXPECT_NE(packed.actions[i].cmd, packed.actions[i - 1].cmd);  // no adjacent equal commands
  }
}

TEST(Compression, MergesATwelveStepStraightRunIntoOneLegalCommand) {
  Pose p{5, 100, 0};
  LatticePlan plan;
  plan.valid = true;
  for (int i = 0; i < 12; ++i) {  // 12 * 10 = 120 cm of forward motion
    const Pose q = propagate(p, Cmd::F, 10, cfg);
    plan.actions.push_back({Cmd::F, 10, p, q, 10.0});
    p = q;
  }
  plan.final_pose = p;
  plan.total_distance_cm = 120;
  plan.total_cost = path_cost_cm(plan.actions, cfg);
  const LatticePlan packed = compress_lattice_plan(plan, {}, cfg);
  ASSERT_EQ(packed.actions.size(), 1u);
  EXPECT_EQ(packed.actions[0].value, 120);
  EXPECT_TRUE(value_is_legal(packed.actions[0].cmd, packed.actions[0].value));
}

TEST(Compression, RunsLongerThanTheStmLimitAreSplitNotRejected) {
  Pose p{5, 100, 0};
  LatticePlan plan;
  plan.valid = true;
  // Arena is only 200 cm, so use a synthetic config with a bigger arena for 1100 cm.
  PlannerConfig big = cfg;
  big.arena_cm = 5000;
  for (int i = 0; i < 110; ++i) {
    const Pose q = propagate(p, Cmd::F, 10, big);
    plan.actions.push_back({Cmd::F, 10, p, q, 10.0});
    p = q;
  }
  plan.final_pose = p;
  plan.total_distance_cm = 1100;
  plan.total_cost = path_cost_cm(plan.actions, big);
  const LatticePlan packed = compress_lattice_plan(plan, {}, big);
  ASSERT_EQ(packed.actions.size(), 2u);  // 990 + 110, each within 1..999
  EXPECT_EQ(packed.actions[0].value, 990);
  EXPECT_EQ(packed.actions[1].value, 110);
}

TEST(Compression, WireStringFormat) {
  EXPECT_EQ(wire_string(Cmd::F, 30), "F,30");
  EXPECT_EQ(wire_string(Cmd::BL, 90), "BL,90");
}
