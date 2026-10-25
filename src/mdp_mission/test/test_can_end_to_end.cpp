// The same mission as test_end_to_end.cpp, run over a simulated CAN bus instead of a UART.
// The planner, executor, driver and device model are unchanged; only the transport differs.
#include <gtest/gtest.h>

#include <cmath>

#include "mdp_mission/can_sim_link.hpp"
#include "mdp_mission/sim_link.hpp"

using namespace mission;

namespace {

const mdp::PlannerConfig cfg;
const mdp::Pose kStart{20, 10, mdp::kPi / 2};
// Same layout as the A_spread_03 benchmark fixture.
const std::vector<mdp::Obstacle> kThree{{1, 30, 100, 'S'}, {2, 100, 20, 'W'}, {3, 130, 50, 'W'}};

mdp::RouteResult plan() {
  mdp::RouteOptions opts;
  opts.time_limit = std::chrono::seconds(30);
  return mdp::optimise_route(kStart, kThree, cfg, mdp::default_leg_planner(kThree, cfg), opts);
}

double dist(const mdp::Pose& a, const mdp::Pose& b) { return std::hypot(a.x - b.x, a.y - b.y); }

template <typename L>
void power_up(L& link) {
  link.bridge().send_imu_get(link.now_s());
  link.advance(0.1);
}

}  // namespace

TEST(CanEndToEnd, TheFullMissionCompletesOverCan) {
  const auto route = plan();
  CanSimLink link(kStart);
  power_up(link);
  const auto r = run_mission(route, link);
  EXPECT_TRUE(r.completed);
  EXPECT_EQ(r.commands_done, r.commands_total);
  EXPECT_FALSE(r.pose_uncertain);
  EXPECT_EQ(r.snapshots.size(), 3u);
}

TEST(CanEndToEnd, TheResultIsIdenticalToTheUartRun) {
  const auto route = plan();
  SimLink uart(kStart);
  CanSimLink can_link(kStart);
  power_up(uart);
  power_up(can_link);
  const auto ru = run_mission(route, uart);
  const auto rc = run_mission(route, can_link);
  ASSERT_TRUE(ru.completed && rc.completed);
  EXPECT_LT(dist(uart.bridge().commanded_pose(), can_link.bridge().commanded_pose()), 1e-9);
  EXPECT_LT(dist(uart.device().true_pose(), can_link.device().true_pose()), 1e-9);
  EXPECT_EQ(ru.commands_done, rc.commands_done);
}

TEST(CanEndToEnd, OdometryOverCanIsAsAccurateAsOverUart) {
  const auto route = plan();
  CanSimLink link(kStart);
  power_up(link);
  const auto r = run_mission(route, link);
  ASSERT_TRUE(r.completed);
  for (const auto& s : r.snapshots) EXPECT_LT(dist(s.odometry, s.commanded), 2.0);
}

TEST(CanEndToEnd, EveryViewingPoseIsReachedWithinTheGoalTolerance) {
  const auto route = plan();
  CanSimLink link(kStart);
  power_up(link);
  const auto r = run_mission(route, link);
  ASSERT_TRUE(r.completed);
  for (const auto& s : r.snapshots) {
    EXPECT_LE(dist(s.commanded, s.target), cfg.lattice_goal_position_tolerance_cm + 1e-6);
  }
}

TEST(CanEndToEnd, TheBusIsLightlyLoadedAtFiveHundredKilobitsPerSecond) {
  const auto route = plan();
  CanSimLink link(kStart);
  power_up(link);
  ASSERT_TRUE(run_mission(route, link).completed);
  EXPECT_LT(link.bus().utilisation(), 0.05);  // telemetry + events are a few kbit/s
  EXPECT_EQ(link.bus().stats().frame_errors, 0);
  EXPECT_EQ(link.bus().stats().ack_errors, 0);
  EXPECT_EQ(link.decoder().dropped(), 0);
}

TEST(CanEndToEnd, BothNodesStayErrorActiveOnAHealthyBus) {
  const auto route = plan();
  CanSimLink link(kStart);
  power_up(link);
  ASSERT_TRUE(run_mission(route, link).completed);
  EXPECT_EQ(link.host_node().state(), can::NodeState::ErrorActive);
  EXPECT_EQ(link.ecu_node().state(), can::NodeState::ErrorActive);
  EXPECT_EQ(link.host_node().tec(), 0);
  EXPECT_EQ(link.ecu_node().tec(), 0);
}

// ---------------------------------------------------------------- faults

// Bit errors on the wire are detected by CRC / stuffing checks and the frame is retransmitted, so
// the mission still completes with exactly the same result. (On the UART, lost bytes could
// abort a mission: see EndToEndFaults.RandomByteLoss...)
TEST(CanEndToEndFaults, RandomBitErrorsAreRecoveredByRetransmission) {
  const auto route = plan();
  CanSimLink clean(kStart);
  power_up(clean);
  ASSERT_TRUE(run_mission(route, clean).completed);

  long total_frame_errors = 0;
  for (unsigned seed = 1; seed <= 10; ++seed) {
    can::Bus::Config bus;
    bus.bit_error_rate = 0.0005;  // about 1 corrupted frame in 20
    bus.seed = seed;
    CanSimLink link(kStart, bus);
    power_up(link);
    const auto r = run_mission(route, link);
    EXPECT_TRUE(r.completed) << "seed " << seed;
    EXPECT_LT(dist(link.bridge().commanded_pose(), clean.bridge().commanded_pose()), 1e-9)
        << "seed " << seed;
    EXPECT_NE(link.host_node().state(), can::NodeState::BusOff) << "seed " << seed;
    total_frame_errors += link.bus().stats().frame_errors;
  }
  EXPECT_GT(total_frame_errors, 0);  // errors really were injected and really were recovered
}

TEST(CanEndToEndFaults, HeavyBitErrorsEndCleanlyInBoundedTime) {
  const auto route = plan();
  for (unsigned seed = 1; seed <= 6; ++seed) {
    can::Bus::Config bus;
    bus.bit_error_rate = 0.01;  // roughly two frames in three are corrupted
    bus.seed = seed;
    CanSimLink link(kStart, bus);
    power_up(link);
    const auto r = run_mission(route, link);
    EXPECT_LT(r.elapsed_s, 600.0) << "seed " << seed;
    if (!r.completed) {
      EXPECT_TRUE(r.failure == stm::Outcome::LinkTimeout || r.failure == stm::Outcome::ErrBusy ||
                  r.failure == stm::Outcome::ErrOther)
          << "seed " << seed << " outcome " << stm::outcome_name(r.failure);
    }
  }
}

TEST(CanEndToEndFaults, ADeadMotorEcuMakesTheHostGoBusOffAndTheMissionAbortCleanly) {
  const auto route = plan();
  CanSimLink link(kStart);
  link.ecu_node().set_enabled(false);  // unplugged: nobody acknowledges the host's frames
  const auto r = run_mission(route, link);
  EXPECT_FALSE(r.completed);
  EXPECT_EQ(r.failure, stm::Outcome::LinkTimeout);
  EXPECT_TRUE(r.pose_uncertain);
  EXPECT_EQ(link.host_node().state(), can::NodeState::BusOff);
  EXPECT_LT(r.elapsed_s, 30.0);
}

TEST(CanEndToEndFaults, AStopFrameBeatsTelemetryOnTheBusAndHaltsTheRobotQuickly) {
  const auto route = plan();
  CanSimLink link(kStart);
  power_up(link);
  const auto flat = flatten(route);
  ASSERT_FALSE(flat.commands.empty());
  link.bridge().send_move(flat.commands[0], link.now_s());
  link.advance(0.6);  // moving, telemetry flowing
  const double sent_at = link.now_s();
  link.bridge().send_stop(sent_at);
  std::optional<stm::CommandResult> r;
  for (int i = 0; i < 300 && !r; ++i) {
    link.advance(0.01);
    r = link.bridge().take_result();
  }
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, stm::Outcome::Stopped);
  EXPECT_LT(link.now_s() - sent_at, 0.4);  // the 150 ms brake window plus bus and polling latency
  EXPECT_FALSE(link.device().moving());
  EXPECT_TRUE(link.bridge().pose_uncertain());
}

TEST(CanEndToEndFaults, StalledMotorsStillGiveMotionTimeoutOverCan) {
  const auto route = plan();
  stm::FaultConfig f;
  f.stall = true;
  CanSimLink link(kStart, {}, f);
  const auto r = run_mission(route, link);
  EXPECT_FALSE(r.completed);
  EXPECT_EQ(r.failure, stm::Outcome::ErrMotionTimeout);
  EXPECT_TRUE(r.pose_uncertain);
}

TEST(CanEndToEndFaults, ImuFailureIsRetriedOnceThenAbortedOverCan) {
  const auto route = plan();
  stm::FaultConfig f;
  f.imu_fail = true;
  CanSimLink link(kStart, {}, f);
  const auto r = run_mission(route, link);
  EXPECT_FALSE(r.completed);
  EXPECT_EQ(r.failure, stm::Outcome::ErrImuNotReady);
  EXPECT_EQ(r.retries, 1);
  EXPECT_FALSE(r.pose_uncertain);
}

TEST(CanEndToEndFaults, WheelSlipIsStillVisibleToTheOdometryOverCan) {
  const auto route = plan();
  stm::FaultConfig f;
  f.slip = 0.05;
  CanSimLink link(kStart, {}, f);
  power_up(link);
  ASSERT_TRUE(run_mission(route, link).completed);
  const double commanded_error = dist(link.bridge().commanded_pose(), link.device().true_pose());
  const double odometry_error = dist(link.bridge().odometry().pose, link.device().true_pose());
  EXPECT_GT(commanded_error, 3.0);
  EXPECT_LT(odometry_error, commanded_error);
}
