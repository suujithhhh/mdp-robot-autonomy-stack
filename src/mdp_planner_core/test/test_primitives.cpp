#include <gtest/gtest.h>

#include "mdp_planner_core/primitives.hpp"

using namespace mdp;

namespace {
const PlannerConfig cfg;
}

TEST(Primitives, StraightForwardMovesAlongHeading) {
  const Pose p = propagate({10, 20, 0.0}, Cmd::F, 30, cfg);
  EXPECT_NEAR(p.x, 40.0, 1e-9);
  EXPECT_NEAR(p.y, 20.0, 1e-9);
  EXPECT_DOUBLE_EQ(p.heading, 0.0);
}

TEST(Primitives, ReverseMovesAgainstHeading) {
  const Pose p = propagate({50, 50, kPi / 2}, Cmd::B, 10, cfg);
  EXPECT_NEAR(p.x, 50.0, 1e-9);
  EXPECT_NEAR(p.y, 40.0, 1e-9);
}

// Worked example from PROTOCOL.md: FL 90 from the origin facing +x ends at (R_FL, R_FL, 90 deg).
TEST(Primitives, FrontLeftNinetyMatchesProtocolExample) {
  const Pose p = propagate({0, 0, 0}, Cmd::FL, 90, cfg);
  EXPECT_NEAR(p.x, 24.8, 1e-9);
  EXPECT_NEAR(p.y, 24.8, 1e-9);
  EXPECT_NEAR(p.heading, kPi / 2, 1e-9);
}

TEST(Primitives, EachTurnUsesItsOwnMeasuredRadius) {
  EXPECT_DOUBLE_EQ(turn_radius_cm(Cmd::FL, cfg), 24.8);
  EXPECT_DOUBLE_EQ(turn_radius_cm(Cmd::FR, cfg), 33.7);
  EXPECT_DOUBLE_EQ(turn_radius_cm(Cmd::BL, cfg), 33.6);
  EXPECT_DOUBLE_EQ(turn_radius_cm(Cmd::BR, cfg), 24.5);
}

TEST(Primitives, TurnDirectionsMatchSteeringLockAndTravelDirection) {
  EXPECT_GT(propagate({0, 0, 0}, Cmd::FL, 45, cfg).heading, 0);  // forward, steer left: CCW
  EXPECT_LT(propagate({0, 0, 0}, Cmd::FR, 45, cfg).heading, 0);  // forward, steer right: CW
  // BL shares the steering lock of FR, BR the lock of FL (PROTOCOL.md section 3); reversing with
  // the same lock rotates the heading the opposite way.
  EXPECT_GT(propagate({0, 0, 0}, Cmd::BL, 45, cfg).heading, 0);  // CCW
  EXPECT_LT(propagate({0, 0, 0}, Cmd::BR, 45, cfg).heading, 0);  // CW
}

TEST(Primitives, FullCircleReturnsToStart) {
  for (Cmd c : {Cmd::FL, Cmd::FR, Cmd::BL, Cmd::BR}) {
    const Pose p = propagate({30, 40, 0.3}, c, 360, cfg);
    EXPECT_NEAR(p.x, 30, 1e-9) << cmd_name(c);
    EXPECT_NEAR(p.y, 40, 1e-9) << cmd_name(c);
  }
}

TEST(Primitives, ArcLengthIsRadiusTimesAngle) {
  EXPECT_NEAR(primitive_length_cm(Cmd::FR, 90, cfg), 33.7 * kPi / 2, 1e-9);
  EXPECT_DOUBLE_EQ(primitive_length_cm(Cmd::B, 25, cfg), 25.0);
}

TEST(Primitives, ValueLimitsMatchStmWireFormat) {
  EXPECT_TRUE(value_is_legal(Cmd::F, 1));
  EXPECT_TRUE(value_is_legal(Cmd::F, 999));
  EXPECT_FALSE(value_is_legal(Cmd::F, 0));
  EXPECT_FALSE(value_is_legal(Cmd::F, 1000));
  EXPECT_TRUE(value_is_legal(Cmd::FL, 360));
  EXPECT_FALSE(value_is_legal(Cmd::FL, 361));
}

TEST(Primitives, NormalizeHeadingWrapsToHalfOpenInterval) {
  EXPECT_NEAR(normalize_heading(3 * kPi), -kPi, 1e-9);
  EXPECT_NEAR(normalize_heading(-3 * kPi / 2), kPi / 2, 1e-9);
  EXPECT_DOUBLE_EQ(normalize_heading(0.5), 0.5);
  EXPECT_NEAR(normalize_heading(kPi), -kPi, 1e-12);  // pi wraps to -pi
}

TEST(Primitives, RoundTripForwardThenBackwardRestoresPose) {
  const Pose start{80, 90, 1.1};
  const Pose there = propagate(start, Cmd::F, 40, cfg);
  const Pose back = propagate(there, Cmd::B, 40, cfg);
  EXPECT_NEAR(back.x, start.x, 1e-9);
  EXPECT_NEAR(back.y, start.y, 1e-9);
}
