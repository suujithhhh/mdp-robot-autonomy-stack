#include <gtest/gtest.h>

#include <algorithm>
#include <set>

#include "mdp_planner_core/route.hpp"

using namespace mdp;

namespace {
const PlannerConfig cfg;

// Fake leg planner: cost = straight-line distance, no real search. Makes ordering logic
// testable in microseconds and independent of the lattice.
LegPlanner distance_planner(long* calls = nullptr) {
  return [calls](const Pose& a, const Pose& b, Clock::time_point) {
    if (calls) ++*calls;
    LatticePlan p;
    p.valid = true;
    p.total_cost = std::hypot(a.x - b.x, a.y - b.y);
    p.final_pose = b;
    return p;
  };
}
}  // namespace

TEST(ViewingTarget, CameraEndsUpStandoffFromTheFaceAndFacesIt) {
  const Obstacle o{1, 100, 100, 'S'};
  const Pose t = viewing_target(o, cfg);
  EXPECT_NEAR(t.heading, kPi / 2, 1e-12);  // facing north, into the south face
  // Lens = rear point + 15.25 forward; the lens is 25 cm below the face (y = 100).
  EXPECT_NEAR(t.y + cfg.camera_forward_offset_cm, 100 - cfg.camera_standoff_cm, 1e-9);
  EXPECT_NEAR(t.x, 105.0, 1e-9);  // on the face's centre line
}

TEST(ViewingTarget, AllFourFacesHaveDistinctHeadings) {
  std::set<double> headings;
  for (char f : {'N', 'E', 'S', 'W'}) headings.insert(viewing_target({1, 90, 90, f}, cfg).heading);
  EXPECT_EQ(headings.size(), 4u);
}

TEST(ViewingTarget, InvalidFaceThrows) {
  EXPECT_THROW(viewing_target({1, 0, 0, 'X'}, cfg), std::invalid_argument);
}

TEST(HeldKarp, BoundNeverExceedsTrueShortestVisitOrder) {
  // Brute-force the true minimum path over all orders and compare with the bound.
  const std::vector<Pose> t{{20, 20, 0}, {120, 30, 0}, {60, 150, 0}, {170, 160, 0}};
  const HeldKarpBound hk(t, cfg);
  const Pose from{10, 10, 0};
  std::vector<int> perm{0, 1, 2, 3};
  double best = 1e18;
  do {
    double cost = 0;
    Pose cur = from;
    for (int i : perm) {
      cost += std::hypot(cur.x - t[i].x, cur.y - t[i].y);
      cur = t[i];
    }
    best = std::min(best, cost);
  } while (std::next_permutation(perm.begin(), perm.end()));
  const double lb = hk.lower_bound(from, 0b1111u);
  EXPECT_LE(lb, best + 1e-9);
  EXPECT_GT(lb, 0.0);
}

TEST(HeldKarp, EmptyRemainingSetIsZero) {
  const HeldKarpBound hk({{20, 20, 0}}, cfg);
  EXPECT_EQ(hk.lower_bound({0, 0, 0}, 0u), 0.0);
}

TEST(HeldKarp, MoreRemainingTargetsNeverLowersTheBound) {
  const std::vector<Pose> t{{20, 20, 0}, {120, 30, 0}, {60, 150, 0}};
  const HeldKarpBound hk(t, cfg);
  const Pose from{10, 10, 0};
  EXPECT_LE(hk.lower_bound(from, 0b001u), hk.lower_bound(from, 0b011u) + 1e-12);
  EXPECT_LE(hk.lower_bound(from, 0b011u), hk.lower_bound(from, 0b111u) + 1e-12);
}

TEST(Route, VisitsEveryObstacleExactlyOnce) {
  const std::vector<Obstacle> obs{
      {1, 30, 100, 'S'}, {2, 100, 20, 'W'}, {3, 130, 50, 'W'}, {4, 150, 150, 'S'}};
  const RouteResult r = optimise_route({20, 10, kPi / 2}, obs, cfg, distance_planner());
  ASSERT_TRUE(r.complete);
  std::set<int> ids;
  for (const auto& leg : r.legs) ids.insert(leg.obstacle_id);
  EXPECT_EQ(ids.size(), obs.size());
  EXPECT_EQ(r.legs.size(), obs.size());
}

TEST(Route, FindsTheOptimalOrderOnAKnownLayout) {
  // Targets in a row ahead of the start: with a distance-only cost, nearest-first is optimal.
  const std::vector<Obstacle> obs{{1, 150, 20, 'N'}, {2, 50, 20, 'N'}, {3, 100, 20, 'N'}};
  const RouteResult r = optimise_route({10, 10, 0}, obs, cfg, distance_planner());
  ASSERT_TRUE(r.complete);
  EXPECT_EQ(r.legs[0].obstacle_id, 2);
  EXPECT_EQ(r.legs[1].obstacle_id, 3);
  EXPECT_EQ(r.legs[2].obstacle_id, 1);
}

TEST(Route, HeldKarpPruningDoesNotChangeTheOptimalCost) {
  const std::vector<Obstacle> obs{{1, 30, 100, 'S'},  {2, 100, 20, 'W'}, {3, 130, 50, 'W'},
                                  {4, 150, 150, 'S'}, {5, 60, 160, 'E'}};
  RouteOptions with, without;
  without.use_held_karp_bound = false;
  const RouteResult a = optimise_route({20, 10, kPi / 2}, obs, cfg, distance_planner(), with);
  const RouteResult b = optimise_route({20, 10, kPi / 2}, obs, cfg, distance_planner(), without);
  ASSERT_TRUE(a.complete && b.complete);
  EXPECT_NEAR(a.total_cost, b.total_cost, 1e-9);
}

TEST(Route, HeldKarpPruningPlansFewerLegs) {
  const std::vector<Obstacle> obs{{1, 30, 100, 'S'},  {2, 100, 20, 'W'}, {3, 130, 50, 'W'},
                                  {4, 150, 150, 'S'}, {5, 60, 160, 'E'},  {6, 170, 90, 'W'}};
  RouteOptions with, without;
  without.use_held_karp_bound = false;
  const RouteResult a = optimise_route({20, 10, kPi / 2}, obs, cfg, distance_planner(), with);
  const RouteResult b = optimise_route({20, 10, kPi / 2}, obs, cfg, distance_planner(), without);
  EXPECT_LE(a.stats.leg_plans, b.stats.leg_plans);
  EXPECT_GT(a.stats.bound_prunes, 0);
}

TEST(Route, LegCacheMeansEachDistinctLegIsPlannedOnce) {
  long calls = 0;
  const std::vector<Obstacle> obs{{1, 30, 100, 'S'}, {2, 100, 20, 'W'}, {3, 130, 50, 'W'}};
  const RouteResult r = optimise_route({20, 10, kPi / 2}, obs, cfg, distance_planner(&calls));
  EXPECT_EQ(calls, r.stats.leg_plans);
  EXPECT_LE(r.stats.leg_plans, r.stats.leg_requests);
}

TEST(Route, UnreachableObstacleStillYieldsABestVerifiedPartialRoute) {
  const std::vector<Obstacle> obs{{1, 30, 100, 'S'}, {2, 100, 20, 'W'}};
  const LegPlanner picky = [](const Pose& a, const Pose& b, Clock::time_point) {
    LatticePlan p;
    if (b.y > 50) return p;  // cannot reach the tall target (obstacle 1)
    p.valid = true;
    p.total_cost = std::hypot(a.x - b.x, a.y - b.y);
    p.final_pose = b;
    return p;
  };
  const RouteResult r = optimise_route({20, 10, kPi / 2}, obs, cfg, picky);
  EXPECT_FALSE(r.complete);
  EXPECT_TRUE(r.valid);
  EXPECT_EQ(r.legs.size(), 1u);
}

TEST(Route, ObstacleWithNoSafeViewingPoseFailsFast) {
  // Face W of an obstacle at x = 5 would put the camera outside the arena.
  const std::vector<Obstacle> obs{{1, 5, 100, 'W'}};
  const RouteResult r = optimise_route({100, 100, 0}, obs, cfg, distance_planner());
  EXPECT_FALSE(r.valid);
  EXPECT_EQ(r.reason, "no_safe_viewing_pose");
}

TEST(Route, TimeLimitIsEnforcedAndReturnsPromptly) {
  const std::vector<Obstacle> obs{{1, 30, 100, 'S'}, {2, 100, 20, 'W'}, {3, 130, 50, 'W'}};
  const LegPlanner slow = [](const Pose&, const Pose&, Clock::time_point deadline) {
    while (Clock::now() < deadline) {
    }  // burn the whole budget like a runaway search
    LatticePlan p;
    p.reason = FailReason::Timeout;
    return p;
  };
  RouteOptions opts;
  opts.time_limit = std::chrono::milliseconds(50);
  const auto t0 = Clock::now();
  const RouteResult r = optimise_route({20, 10, kPi / 2}, obs, cfg, slow, opts);
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count();
  EXPECT_TRUE(r.timed_out);
  EXPECT_LT(elapsed, 500);
}

TEST(Route, RealPlannerCompletesASmallRouteWithValidLegs) {
  const std::vector<Obstacle> obs{{1, 30, 100, 'S'}, {2, 100, 20, 'W'}};
  const RouteResult r =
      optimise_route({20, 10, kPi / 2}, obs, cfg, default_leg_planner(obs, cfg));
  ASSERT_TRUE(r.complete);
  EXPECT_GT(r.total_cost, 0.0);
  for (const auto& leg : r.legs) EXPECT_TRUE(leg.plan.valid);
}
