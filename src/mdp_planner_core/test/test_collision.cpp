#include <gtest/gtest.h>

#include "mdp_planner_core/footprint.hpp"
#include "mdp_planner_core/swept.hpp"

using namespace mdp;

namespace {
const PlannerConfig cfg;
const std::vector<Obstacle> kNone;
}  // namespace

TEST(Footprint, CornersAreRearReferencedAndMarginInflated) {
  const auto corners = footprint_corners({100, 100, 0.0}, local_bounds(cfg));
  // rear -3 (0 extent + 3 margin), front +27, half-width 9.5 + 3 = 12.5
  EXPECT_NEAR(corners[0].x, 97.0, 1e-9);
  EXPECT_NEAR(corners[0].y, 87.5, 1e-9);
  EXPECT_NEAR(corners[2].x, 127.0, 1e-9);
  EXPECT_NEAR(corners[2].y, 112.5, 1e-9);
}

TEST(Footprint, OpenArenaCentreIsSafe) { EXPECT_TRUE(pose_is_safe({100, 100, 0}, kNone, cfg)); }

TEST(Footprint, TouchingTheWallIsAllowedButCrossingIsNot) {
  // Rear margin is 3 cm, so x = 3 puts the rear edge exactly on the wall.
  EXPECT_TRUE(pose_is_safe({3.0, 100, 0}, kNone, cfg));
  EXPECT_FALSE(pose_is_safe({2.9, 100, 0}, kNone, cfg));
}

TEST(Footprint, FrontOverlappingObstacleCollides) {
  const std::vector<Obstacle> obs{{1, 130, 95, 'W'}};
  EXPECT_FALSE(pose_is_safe({110, 100, 0}, obs, cfg));
  EXPECT_TRUE(pose_is_safe({50, 100, 0}, obs, cfg));
}

TEST(Footprint, RotatedFootprintUsesTheRobotAxes) {
  // A diagonal robot whose bounding box overlaps the obstacle but whose body does not.
  const std::vector<Obstacle> obs{{1, 100, 60, 'S'}};
  const Pose diag{60, 60, kPi / 4};
  EXPECT_TRUE(pose_is_safe(diag, obs, cfg));
}

TEST(Footprint, MarginIsAppliedExactlyOnce) {
  const std::vector<Obstacle> obs{{1, 130, 95, 'W'}};
  // front edge = x + 24 + 3, the obstacle starts at 130: collision when x + 27 > 130.
  EXPECT_TRUE(pose_is_safe({102.9, 100, 0}, obs, cfg));
  EXPECT_FALSE(pose_is_safe({103.1, 100, 0}, obs, cfg));
}

TEST(Swept, SamplesIncludeExactStartAndEnd) {
  const Pose start{40, 40, 0.0};
  const Samples s = sample_primitive(start, Cmd::F, 20, cfg);
  EXPECT_EQ(s.poses.front(), start);
  EXPECT_EQ(s.poses.back(), propagate(start, Cmd::F, 20, cfg));
}

TEST(Swept, StraightSamplesAreNoMoreThanOneStepApart) {
  const Samples s = sample_primitive({40, 40, 0.0}, Cmd::F, 50, cfg);
  for (std::size_t i = 1; i < s.poses.size(); ++i) {
    EXPECT_LE(std::hypot(s.poses[i].x - s.poses[i - 1].x, s.poses[i].y - s.poses[i - 1].y),
              1.0 + 1e-9);
  }
}

TEST(Swept, TurnEndpointEqualsPropagate) {
  const Pose start{60, 60, 0.7};
  for (Cmd c : {Cmd::FL, Cmd::FR, Cmd::BL, Cmd::BR}) {
    const Samples s = sample_primitive(start, c, 90, cfg);
    const Pose end = propagate(start, c, 90, cfg);
    EXPECT_NEAR(s.poses.back().x, end.x, 1e-9) << cmd_name(c);
    EXPECT_NEAR(s.poses.back().y, end.y, 1e-9) << cmd_name(c);
  }
}

TEST(Swept, DetectsObstacleBetweenEndpoints) {
  // The robot starts and ends clear of the obstacle but drives straight through it.
  const std::vector<Obstacle> obs{{1, 90, 95, 'W'}};
  const auto rects = make_rects(obs, cfg);
  EXPECT_FALSE(safe_primitive_samples({20, 100, 0}, Cmd::F, 150, rects, cfg).has_value());
  EXPECT_TRUE(safe_primitive_samples({20, 100, 0}, Cmd::F, 40, rects, cfg).has_value());
}

TEST(Swept, TurnSweepingIntoWallIsRejected) {
  const auto rects = make_rects(kNone, cfg);
  EXPECT_FALSE(safe_primitive_samples({10, 100, kPi}, Cmd::FL, 90, rects, cfg).has_value());
  EXPECT_TRUE(safe_primitive_samples({100, 100, 0}, Cmd::FL, 90, rects, cfg).has_value());
}

TEST(Swept, EndOnlyVariantAgreesWithFullSamples) {
  const auto rects = make_rects(kNone, cfg);
  const auto full = safe_primitive_samples({100, 100, 0.4}, Cmd::BR, 60, rects, cfg);
  const auto end = safe_primitive_end({100, 100, 0.4}, Cmd::BR, 60, rects, cfg);
  ASSERT_TRUE(full && end);
  EXPECT_EQ(full->back(), *end);
}
