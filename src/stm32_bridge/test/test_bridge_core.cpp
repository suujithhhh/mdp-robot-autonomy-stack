#include <gtest/gtest.h>

#include "stm32_bridge/bridge_core.hpp"
#include "virtual_stm32/virtual_stm.hpp"

using namespace stm;

namespace {

const mdp::PlannerConfig cfg;
const mdp::Pose kStart{50, 50, 0.0};

MoveCmd mv(mdp::Cmd c, int v) { return {c, v}; }

// Minimal harness: a bridge wired to a virtual device with a simulated clock.
struct Rig {
  explicit Rig(const FaultConfig& f = {}) : dev(kStart, f), bridge(kStart) {}

  void run(double seconds) {
    for (double s = 0; s < seconds; s += 0.01) {
      dev.receive(bridge.take_tx());
      dev.step(0.01);
      t += 0.01;
      bridge.on_bytes(dev.take_output(), t);
      bridge.poll(t);
    }
  }
  // Run until the bridge reports a result for the command in flight.
  std::optional<CommandResult> wait(double max_s = 60.0) {
    for (double s = 0; s < max_s; s += 0.01) {
      run(0.01);
      if (auto r = bridge.take_result()) return r;
    }
    return std::nullopt;
  }

  VirtualStm dev;
  BridgeCore bridge;
  double t{0.0};
};

}  // namespace

TEST(Bridge, SendMoveTransmitsTheWireFormat) {
  BridgeCore b(kStart);
  ASSERT_TRUE(b.send_move(mv(mdp::Cmd::FL, 90), 0.0));
  EXPECT_EQ(b.take_tx(), "MOV,FL,90\n");
}

TEST(Bridge, OnlyOneCommandInFlight) {
  BridgeCore b(kStart);
  ASSERT_TRUE(b.send_move(mv(mdp::Cmd::F, 10), 0.0));
  b.take_tx();
  EXPECT_FALSE(b.send_move(mv(mdp::Cmd::B, 10), 0.1));  // refused locally
  EXPECT_TRUE(b.take_tx().empty());                      // and nothing was transmitted
}

TEST(Bridge, AckResolvesTheCommandAndAdvancesTheCommandedPose) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 30), 0.0);
  b.on_bytes("ACK\n", 2.0);
  const auto r = b.take_result();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, Outcome::Ack);
  EXPECT_EQ(r->wire, "MOV,F,30");
  EXPECT_NEAR(b.commanded_pose().x, 80.0, 1e-9);
  EXPECT_FALSE(b.busy());
}

TEST(Bridge, ErrorRepliesAreMappedToOutcomes) {
  struct Case {
    const char* reply;
    Outcome outcome;
  } cases[] = {{"ERR,BUSY\n", Outcome::ErrBusy},
               {"ERR,BAD_CMD\n", Outcome::ErrBadCmd},
               {"ERR,IMU_NOT_READY\n", Outcome::ErrImuNotReady},
               {"ERR,MOTION_TIMEOUT\n", Outcome::ErrMotionTimeout},
               {"ERR,PID_NOT_SET\n", Outcome::ErrOther}};
  for (const auto& c : cases) {
    BridgeCore b(kStart);
    b.send_move(mv(mdp::Cmd::F, 10), 0.0);
    b.on_bytes(c.reply, 1.0);
    const auto r = b.take_result();
    ASSERT_TRUE(r.has_value()) << c.reply;
    EXPECT_EQ(r->outcome, c.outcome) << c.reply;
  }
}

TEST(Bridge, ErrorDoesNotAdvanceTheCommandedPose) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 30), 0.0);
  b.on_bytes("ERR,BUSY\n", 1.0);
  EXPECT_DOUBLE_EQ(b.commanded_pose().x, 50.0);
}

TEST(Bridge, MotionTimeoutMarksThePoseUncertain) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 10), 0.0);
  b.on_bytes("MM,EV:START,D:F\nERR,MOTION_TIMEOUT\n", 6.0);
  EXPECT_TRUE(b.pose_uncertain());
}

TEST(Bridge, ImuNotReadyBeforeStartLeavesThePoseKnown) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 10), 0.0);
  b.on_bytes("ERR,IMU_NOT_READY\n", 0.1);
  EXPECT_FALSE(b.pose_uncertain());  // nothing moved: a retry is allowed
}

TEST(Bridge, ImuNotReadyAfterStartMakesThePoseUncertain) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 10), 0.0);
  b.on_bytes("MM,EV:START,D:F\nERR,IMU_NOT_READY\n", 0.5);
  EXPECT_TRUE(b.pose_uncertain());
}

TEST(Bridge, TelemetryAndEventsAreNeverTreatedAsReplies) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 10), 0.0);
  b.on_bytes("MM,EV:START,D:F\nMM,TEL,T:100,L:75,R:75,H:0\n", 0.2);
  EXPECT_FALSE(b.take_result().has_value());
  EXPECT_TRUE(b.busy());
}

TEST(Bridge, UnexpectedTerminalRepliesAreCountedNotApplied) {
  BridgeCore b(kStart);
  b.on_bytes("ACK\nERR,BUSY\nACK,STOPPED\n", 1.0);
  EXPECT_EQ(b.unexpected_replies(), 3);
  EXPECT_FALSE(b.take_result().has_value());
  EXPECT_DOUBLE_EQ(b.commanded_pose().x, 50.0);
}

TEST(Bridge, GarbageLinesAreCountedAndIgnored) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 10), 0.0);
  b.on_bytes("\x01\x02garbage\nACK\n", 1.0);
  EXPECT_EQ(b.unparsed_lines(), 1);
  EXPECT_TRUE(b.take_result().has_value());
}

TEST(Bridge, ReplyArrivingInSmallChunksIsStillMatched) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 10), 0.0);
  for (char ch : std::string("ACK\n")) b.on_bytes(std::string(1, ch), 1.0);
  const auto r = b.take_result();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, Outcome::Ack);
}

// ---------------------------------------------------------------- link timeout

TEST(Bridge, SilenceBecomesALinkTimeoutAfterTheFirmwareDeadlinePlusMargin) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 20), 0.0);  // firmware deadline 5 + 3 = 8 s, margin 1.5 s
  b.take_tx();
  b.poll(9.0);
  EXPECT_FALSE(b.take_result().has_value());
  b.poll(9.6);
  const auto r = b.take_result();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, Outcome::LinkTimeout);
  EXPECT_TRUE(r->pose_uncertain);
}

TEST(Bridge, LinkTimeoutSendsAConfirmingStopAndNeverResendsTheCommand) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 20), 0.0);
  b.take_tx();
  b.poll(20.0);
  EXPECT_EQ(b.take_tx(), "STOP\n");  // only a STOP: the MOV is never retransmitted
  b.poll(21.0);
  b.poll(22.0);
  EXPECT_TRUE(b.take_tx().empty());
}

TEST(Bridge, ConfirmingStopReplyIsAbsorbedNotReportedAsANewResult) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 20), 0.0);
  b.poll(20.0);
  b.take_result();  // the LinkTimeout
  b.on_bytes("ACK,STOPPED\n", 20.1);
  EXPECT_FALSE(b.take_result().has_value());
  EXPECT_EQ(b.unexpected_replies(), 0);
  EXPECT_FALSE(b.busy());
}

TEST(Bridge, NoNewMoveWhileAStopIsOutstanding) {
  BridgeCore b(kStart);
  b.send_stop(0.0);
  EXPECT_FALSE(b.send_move(mv(mdp::Cmd::F, 10), 0.1));
}

TEST(Bridge, StopCrossingAMovementReportsStoppedAndMarksThePoseUncertain) {
  BridgeCore b(kStart);
  b.send_move(mv(mdp::Cmd::F, 50), 0.0);
  b.send_stop(0.5);
  b.on_bytes("MM,EV:START,D:F\nMM,EV:END,D:F,T:500,RESULT:STOPPED,DL:300,DR:300\nACK,STOPPED\n", 0.7);
  const auto r = b.take_result();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, Outcome::Stopped);
  EXPECT_TRUE(b.pose_uncertain());
}

TEST(Bridge, StopAtIdleLeavesAKnownPoseKnown) {
  BridgeCore b(kStart);
  b.send_stop(0.0);
  b.on_bytes("ACK,STOPPED\n", 0.03);
  const auto r = b.take_result();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->wire, "STOP");
  EXPECT_FALSE(b.pose_uncertain());
}

TEST(Bridge, DuplicateStopsAreCoalescedLocally) {
  BridgeCore b(kStart);
  b.send_stop(0.0);
  b.send_stop(0.1);
  EXPECT_EQ(b.take_tx(), "STOP\n");
}

// ---------------------------------------------------------------- against the virtual device

TEST(BridgeWithDevice, StraightMoveAcksAndOdometryMatchesTheTruth) {
  Rig rig;
  rig.bridge.send_imu_get(0.0);
  rig.run(0.1);
  rig.bridge.send_move(mv(mdp::Cmd::F, 60), rig.t);
  const auto r = rig.wait();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, Outcome::Ack);
  EXPECT_NEAR(rig.bridge.odometry().pose.x, rig.dev.true_pose().x, 0.5);
  EXPECT_NEAR(rig.bridge.odometry().pose.y, rig.dev.true_pose().y, 0.5);
  EXPECT_NEAR(rig.bridge.commanded_pose().x, 110.0, 1e-6);
  EXPECT_GE(rig.bridge.odometry().telemetry_count, 10);
}

TEST(BridgeWithDevice, TurnOdometryTracksTheTrueHeading) {
  Rig rig;
  rig.bridge.send_imu_get(0.0);
  rig.run(0.1);
  rig.bridge.send_move(mv(mdp::Cmd::FL, 90), rig.t);
  ASSERT_TRUE(rig.wait().has_value());
  rig.run(0.1);  // the post-ACK IMU,GET reply fixes the heading for the last telemetry interval
  EXPECT_NEAR(rig.bridge.odometry().pose.heading, rig.dev.true_pose().heading, 0.01);
  EXPECT_NEAR(rig.bridge.odometry().pose.x, rig.dev.true_pose().x, 1.5);
  EXPECT_NEAR(rig.bridge.odometry().pose.y, rig.dev.true_pose().y, 1.5);
}

TEST(BridgeWithDevice, SlipMakesOdometryDisagreeWithTheCommandedPoseButNotWithTheTruth) {
  FaultConfig f;
  f.slip = 0.1;
  Rig rig(f);
  rig.bridge.send_imu_get(0.0);
  rig.run(0.1);
  rig.bridge.send_move(mv(mdp::Cmd::F, 100), rig.t);
  ASSERT_TRUE(rig.wait().has_value());
  // The command was ACKed, so the dead-reckoned estimate says x = 150; the wheels say otherwise.
  EXPECT_NEAR(rig.bridge.commanded_pose().x, 150.0, 1e-6);
  EXPECT_NEAR(rig.bridge.odometry().pose.x, rig.dev.true_pose().x, 1.0);
  EXPECT_GT(std::fabs(rig.bridge.commanded_pose().x - rig.dev.true_pose().x), 5.0);
}

TEST(BridgeWithDevice, ReadsOneByteAtATimeProduceTheSameResult) {
  Rig rig;
  rig.bridge.send_move(mv(mdp::Cmd::F, 20), 0.0);
  std::optional<CommandResult> result;
  for (int i = 0; i < 6000 && !result; ++i) {
    rig.dev.receive(rig.bridge.take_tx());
    rig.dev.step(0.01);
    rig.t += 0.01;
    for (char ch : rig.dev.take_output()) rig.bridge.on_bytes(std::string(1, ch), rig.t);
    rig.bridge.poll(rig.t);
    result = rig.bridge.take_result();
  }
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->outcome, Outcome::Ack);
}

TEST(BridgeWithDevice, StalledMotorsGiveMotionTimeoutNotAHang) {
  FaultConfig f;
  f.stall = true;
  Rig rig(f);
  rig.bridge.send_move(mv(mdp::Cmd::F, 20), 0.0);
  const auto r = rig.wait();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, Outcome::ErrMotionTimeout);
  EXPECT_TRUE(rig.bridge.pose_uncertain());
  EXPECT_LT(rig.t, 9.0);
}

TEST(BridgeWithDevice, LostTerminalReplyBecomesALinkTimeoutThenARecoveredLink) {
  FaultConfig f;
  f.drop_rate = 1.0;  // every byte from the device is lost
  Rig rig(f);
  rig.bridge.send_move(mv(mdp::Cmd::F, 10), 0.0);
  const auto r = rig.wait();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, Outcome::LinkTimeout);
  EXPECT_TRUE(r->pose_uncertain);
  rig.run(3.0);  // the confirming STOP times out; the bridge stops waiting and is usable again
  EXPECT_FALSE(rig.bridge.busy());
}

TEST(BridgeWithDevice, StopDuringMotionIsReportedAsStopped) {
  Rig rig;
  rig.bridge.send_move(mv(mdp::Cmd::F, 100), 0.0);
  rig.run(1.0);
  rig.bridge.send_stop(rig.t);
  const auto r = rig.wait();
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->outcome, Outcome::Stopped);
  EXPECT_TRUE(rig.bridge.pose_uncertain());
  EXPECT_FALSE(rig.dev.moving());
}
