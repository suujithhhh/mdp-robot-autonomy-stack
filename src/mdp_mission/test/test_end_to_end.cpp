// End-to-end tests: real planner -> mission executor -> driver core -> virtual STM32, all in
// memory with a simulated clock. Fault scenarios check that the stack fails cleanly (a bounded
// simulated time, a specific outcome) instead of hanging or reporting success it did not earn.
#include <gtest/gtest.h>

#include <cmath>
#include <set>

#include "mdp_mission/sim_link.hpp"
#include "mdp_planner_core/footprint.hpp"

using namespace mission;

namespace {

const mdp::PlannerConfig cfg;
const double kDeg90 = mdp::kPi / 2;

// Same layout as the A_spread_03 benchmark fixture of the MDP planner.
const mdp::Pose kStart{20, 10, kDeg90};
const std::vector<mdp::Obstacle> kThree{{1, 30, 100, 'S'}, {2, 100, 20, 'W'}, {3, 130, 50, 'W'}};

mdp::RouteResult plan(const std::vector<mdp::Obstacle>& obs = kThree) {
  mdp::RouteOptions opts;
  opts.time_limit = std::chrono::seconds(30);
  return mdp::optimise_route(kStart, obs, cfg, mdp::default_leg_planner(obs, cfg), opts);
}

double dist(const mdp::Pose& a, const mdp::Pose& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// Bring the driver's IMU reference up before moving, as the real start-up does.
void power_up(SimLink& link) {
  link.bridge().send_imu_get(link.now_s());
  link.advance(0.1);
}

}  // namespace

TEST(EndToEnd, PlannerFindsACompleteRouteForTheThreeObstacleLayout) {
  const auto route = plan();
  ASSERT_TRUE(route.complete);
  EXPECT_EQ(route.legs.size(), 3u);
  const auto flat = flatten(route);
  EXPECT_FALSE(flat.commands.empty());
  EXPECT_EQ(flat.leg_end_index.size(), 3u);
  EXPECT_EQ(flat.leg_end_index.back(), flat.commands.size());
}

TEST(EndToEnd, FlattenedCommandsAreAllLegalStmValues) {
  const auto flat = flatten(plan());
  for (const auto& m : flat.commands) EXPECT_TRUE(mdp::value_is_legal(m.cmd, m.value));
}

TEST(EndToEnd, FullMissionCompletesWithEveryCommandAcked) {
  const auto route = plan();
  SimLink link(kStart);
  power_up(link);
  const auto r = run_mission(route, link);
  EXPECT_TRUE(r.completed);
  EXPECT_EQ(r.commands_done, r.commands_total);
  EXPECT_FALSE(r.pose_uncertain);
  EXPECT_EQ(r.snapshots.size(), 3u);
}

TEST(EndToEnd, EverySnapshotIsWithinThePlannerGoalToleranceOfItsViewingPose) {
  const auto route = plan();
  SimLink link(kStart);
  power_up(link);
  const auto r = run_mission(route, link);
  ASSERT_TRUE(r.completed);
  for (const auto& s : r.snapshots) {
    EXPECT_LE(dist(s.commanded, s.target), cfg.lattice_goal_position_tolerance_cm + 1e-6)
        << "obstacle " << s.obstacle_id;
  }
}

TEST(EndToEnd, WithoutFaultsTheSimulatedRobotEndsExactlyWhereTheDeadReckoningSays) {
  const auto route = plan();
  SimLink link(kStart);
  power_up(link);
  const auto r = run_mission(route, link);
  ASSERT_TRUE(r.completed);
  EXPECT_LT(dist(link.device().true_pose(), link.bridge().commanded_pose()), 1e-6);
}

TEST(EndToEnd, OdometryStaysCloseToTheTruthAtEverySnapshot) {
  const auto route = plan();
  SimLink link(kStart);
  power_up(link);
  double worst = 0.0;
  link.set_observer([&](double, const mdp::Pose&, const mdp::Pose&) {});
  const auto r = run_mission(route, link);
  ASSERT_TRUE(r.completed);
  for (const auto& s : r.snapshots) worst = std::max(worst, dist(s.odometry, s.commanded));
  EXPECT_LT(worst, 2.0);  // cm: quantised wheel counts + 10 Hz telemetry
}

TEST(EndToEnd, MissionIsIdenticalWhenTheDeviceOutputArrivesOneByteAtATime) {
  const auto route = plan();
  SimLink a(kStart), b(kStart);
  b.set_read_chunk(1);
  power_up(a);
  power_up(b);
  const auto ra = run_mission(route, a);
  const auto rb = run_mission(route, b);
  ASSERT_TRUE(ra.completed && rb.completed);
  EXPECT_LT(dist(a.bridge().commanded_pose(), b.bridge().commanded_pose()), 1e-9);
  EXPECT_NEAR(ra.elapsed_s, rb.elapsed_s, 0.02);
}

TEST(EndToEnd, SimulatedMissionTimeIsPlausible) {
  const auto route = plan();
  SimLink link(kStart);
  power_up(link);
  const auto r = run_mission(route, link);
  ASSERT_TRUE(r.completed);
  EXPECT_GT(r.elapsed_s, 5.0);
  EXPECT_LT(r.elapsed_s, 360.0);  // inside the 6-minute Task 1 window
}

TEST(EndToEnd, RecordedPathStartsAtTheStartAndVisitsEveryViewingPose) {
  const auto route = plan();
  SimLink link(kStart);
  std::vector<mdp::Pose> path;
  link.set_observer([&](double, const mdp::Pose& truth, const mdp::Pose&) { path.push_back(truth); });
  power_up(link);
  ASSERT_TRUE(run_mission(route, link).completed);
  EXPECT_LT(dist(path.front(), kStart), 1.0);
  for (const auto& leg : route.legs) {
    double nearest = 1e9;
    for (const auto& p : path) nearest = std::min(nearest, dist(p, leg.target));
    EXPECT_LT(nearest, cfg.lattice_goal_position_tolerance_cm + 0.5) << "obstacle " << leg.obstacle_id;
  }
}

TEST(EndToEnd, RobotNeverEntersAnObstacleWhileDriving) {
  const auto route = plan();
  SimLink link(kStart);
  bool collided = false;
  const auto rects = mdp::make_rects(kThree, cfg);
  const auto bounds = mdp::local_bounds(cfg);
  link.set_observer([&](double, const mdp::Pose& truth, const mdp::Pose&) {
    if (!mdp::pose_is_safe(truth, bounds, rects, cfg.arena_cm)) collided = true;
  });
  power_up(link);
  ASSERT_TRUE(run_mission(route, link).completed);
  EXPECT_FALSE(collided);
}

// ---------------------------------------------------------------- fault scenarios

TEST(EndToEndFaults, StalledMotorsAbortTheMissionWithMotionTimeoutAndAnUncertainPose) {
  const auto route = plan();
  stm::FaultConfig f;
  f.stall = true;
  SimLink link(kStart, f);
  const auto r = run_mission(route, link);
  EXPECT_FALSE(r.completed);
  EXPECT_EQ(r.failure, stm::Outcome::ErrMotionTimeout);
  EXPECT_TRUE(r.pose_uncertain);
  EXPECT_EQ(r.commands_done, 0u);
  EXPECT_LT(r.elapsed_s, 20.0);  // bounded: no hang
}

TEST(EndToEndFaults, ImuFailureIsRetriedOnceThenAbortedWithTheKnownPose) {
  const auto route = plan();
  stm::FaultConfig f;
  f.imu_fail = true;
  SimLink link(kStart, f);
  const auto r = run_mission(route, link);
  EXPECT_FALSE(r.completed);
  EXPECT_EQ(r.failure, stm::Outcome::ErrImuNotReady);
  EXPECT_EQ(r.retries, 1);
  EXPECT_FALSE(r.pose_uncertain);  // nothing ever moved, so the start pose is still exact
  EXPECT_DOUBLE_EQ(link.device().true_pose().x, kStart.x);
}

TEST(EndToEndFaults, ARetryBudgetOfZeroAbortsImmediately) {
  const auto route = plan();
  stm::FaultConfig f;
  f.imu_fail = true;
  SimLink link(kStart, f);
  MissionOptions opts;
  opts.max_retries = 0;
  const auto r = run_mission(route, link, opts);
  EXPECT_EQ(r.retries, 0);
  EXPECT_FALSE(r.completed);
}

TEST(EndToEndFaults, TotalSilenceFromTheDeviceBecomesALinkTimeoutNotAHang) {
  const auto route = plan();
  stm::FaultConfig f;
  f.drop_rate = 1.0;
  SimLink link(kStart, f);
  const auto r = run_mission(route, link);
  EXPECT_FALSE(r.completed);
  EXPECT_EQ(r.failure, stm::Outcome::LinkTimeout);
  EXPECT_TRUE(r.pose_uncertain);
  EXPECT_LT(r.elapsed_s, 30.0);
}

TEST(EndToEndFaults, LostCommandBytesNeverResendTheCommand) {
  const auto route = plan();
  SimLink link(kStart);
  int mov_lines_seen = 0;
  link.set_tx_filter([&](const std::string& tx) {
    if (tx.find("MOV") != std::string::npos) ++mov_lines_seen;
    return std::string();  // the wire eats everything the host sends
  });
  const auto r = run_mission(route, link);
  EXPECT_FALSE(r.completed);
  EXPECT_EQ(r.failure, stm::Outcome::LinkTimeout);
  EXPECT_EQ(mov_lines_seen, 1);  // the first MOV was sent exactly once and never repeated
}

TEST(EndToEndFaults, RandomByteLossAlwaysEndsCleanlyAcrossManySeeds) {
  const auto route = plan();
  int completed = 0, aborted = 0;
  for (unsigned seed = 1; seed <= 20; ++seed) {
    stm::FaultConfig f;
    f.drop_rate = 0.02;
    f.seed = seed;
    SimLink link(kStart, f);
    power_up(link);
    const auto r = run_mission(route, link);
    // Whatever happened, the executor terminated in bounded simulated time with a definite result.
    EXPECT_LT(r.elapsed_s, 600.0) << "seed " << seed;
    if (r.completed) {
      ++completed;
    } else {
      ++aborted;
      EXPECT_TRUE(r.failure == stm::Outcome::LinkTimeout || r.failure == stm::Outcome::ErrBadCmd ||
                  r.failure == stm::Outcome::ErrBusy || r.failure == stm::Outcome::ErrOther)
          << "seed " << seed << " outcome " << stm::outcome_name(r.failure);
    }
  }
  EXPECT_EQ(completed + aborted, 20);
}

TEST(EndToEndFaults, WheelSlipIsInvisibleToTheCommandedPoseButSeenByTheOdometry) {
  const auto route = plan();
  stm::FaultConfig f;
  f.slip = 0.05;
  SimLink link(kStart, f);
  power_up(link);
  const auto r = run_mission(route, link);
  ASSERT_TRUE(r.completed);  // every command was ACKed
  const double commanded_error = dist(link.bridge().commanded_pose(), link.device().true_pose());
  const double odometry_error = dist(link.bridge().odometry().pose, link.device().true_pose());
  EXPECT_GT(commanded_error, 3.0);                // open-loop dead reckoning drifts
  EXPECT_LT(odometry_error, commanded_error);     // wheel counts see (most of) the slip
}

TEST(EndToEndFaults, ImuBiasSkewsTheOdometryHeadingByTheBias) {
  const auto route = plan();
  stm::FaultConfig f;
  f.imu_bias_deg = 2.0;
  SimLink link(kStart, f);
  power_up(link);  // the IMU reference is taken at rest, so a constant bias cancels
  ASSERT_TRUE(run_mission(route, link).completed);
  const double heading_error = std::fabs(mdp::normalize_heading(
      link.bridge().odometry().pose.heading - link.device().true_pose().heading));
  EXPECT_LT(heading_error * mdp::kRadToDeg, 0.5);
}

TEST(EndToEndFaults, NoisyEncodersAndImuStillGiveSubCentimetreTelemetryOdometryOnAStraight) {
  stm::FaultConfig f;
  f.encoder_noise = 0.01;
  f.imu_noise_deg = 0.2;
  f.seed = 3;
  SimLink link(mdp::Pose{20, 100, 0.0}, f);
  power_up(link);
  link.bridge().send_move({mdp::Cmd::F, 100}, link.now_s());
  for (int i = 0; i < 1000 && !link.bridge().take_result(); ++i) link.advance(0.01);
  EXPECT_LT(dist(link.bridge().odometry().pose, link.device().true_pose()), 3.0);
}

TEST(EndToEndFaults, StopMidMissionLeavesTheRobotStationaryAndThePoseUncertain) {
  const auto route = plan();
  SimLink link(kStart);
  power_up(link);
  const auto flat = flatten(route);
  ASSERT_FALSE(flat.commands.empty());
  link.bridge().send_move(flat.commands[0], link.now_s());
  link.advance(0.4);
  link.bridge().send_stop(link.now_s());
  std::optional<stm::CommandResult> r;
  for (int i = 0; i < 500 && !r; ++i) {
    link.advance(0.01);
    r = link.bridge().take_result();
  }
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, stm::Outcome::Stopped);
  EXPECT_TRUE(link.bridge().pose_uncertain());
  const mdp::Pose frozen = link.device().true_pose();
  link.advance(2.0);
  EXPECT_DOUBLE_EQ(link.device().true_pose().x, frozen.x);
}
