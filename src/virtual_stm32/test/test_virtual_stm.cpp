#include <gtest/gtest.h>

#include <algorithm>

#include "virtual_stm32/virtual_stm.hpp"

using namespace stm;

namespace {

const mdp::PlannerConfig cfg;
const mdp::Pose kStart{50, 50, 0.0};

std::vector<std::string> lines_of(const std::string& out) {
  LineFramer f(kMaxHostLineChars);  // device output includes lines longer than a command
  return f.feed(out);
}

// Lines that are terminal replies, in order.
std::vector<std::string> terminals(const std::vector<std::string>& lines) {
  std::vector<std::string> t;
  for (const auto& l : lines) {
    const auto r = parse_reply(l);
    if (r && is_terminal(*r)) t.push_back(l);
  }
  return t;
}

// Run the device for `seconds` and collect everything it printed.
std::vector<std::string> run(VirtualStm& d, double seconds) {
  std::string out;
  for (double t = 0; t < seconds; t += 0.01) {
    d.step(0.01);
    out += d.take_output();
  }
  return lines_of(out);
}

}  // namespace

TEST(VirtualStm, StraightMoveEndsWithExactlyOneAckAndTheIdealPose) {
  VirtualStm d(kStart);
  d.receive("MOV,F,30\n");
  const auto lines = run(d, 5.0);
  const auto t = terminals(lines);
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0], "ACK");
  EXPECT_NEAR(d.true_pose().x, 80.0, 1e-6);
  EXPECT_NEAR(d.true_pose().y, 50.0, 1e-6);
  EXPECT_FALSE(d.moving());
}

TEST(VirtualStm, TurnEndsAtThePrimitivePose) {
  VirtualStm d(kStart);
  d.receive("MOV,FL,90\n");
  run(d, 5.0);
  const mdp::Pose expected = mdp::propagate(kStart, mdp::Cmd::FL, 90, cfg);
  EXPECT_NEAR(d.true_pose().x, expected.x, 1e-6);
  EXPECT_NEAR(d.true_pose().y, expected.y, 1e-6);
  EXPECT_NEAR(d.true_pose().heading, expected.heading, 1e-9);
}

TEST(VirtualStm, StartAndEndEventsBracketTheMovementBeforeTheAck) {
  VirtualStm d(kStart);
  d.receive("MOV,F,10\n");
  const auto lines = run(d, 5.0);
  ASSERT_GE(lines.size(), 3u);
  EXPECT_EQ(lines.front(), "MM,EV:START,D:F");
  EXPECT_EQ(lines.back(), "ACK");
  EXPECT_EQ(lines[lines.size() - 2].rfind("MM,EV:END,D:F", 0), 0u);
}

TEST(VirtualStm, SecondMoveWhileBusyIsRefusedWithBusyAndDoesNotDisturbTheFirst) {
  VirtualStm d(kStart);
  d.receive("MOV,F,50\n");
  d.step(0.2);
  d.receive("MOV,B,10\n");
  const auto lines = run(d, 6.0);
  const auto t = terminals(lines);
  ASSERT_EQ(t.size(), 2u);
  EXPECT_EQ(t[0], "ERR,BUSY");
  EXPECT_EQ(t[1], "ACK");
  EXPECT_NEAR(d.true_pose().x, 100.0, 1e-6);  // only the first command ran
}

TEST(VirtualStm, UnparsableLineGetsBadCmd) {
  VirtualStm d(kStart);
  d.receive("HELLO\nMOV,F,\nmov,f,1\n");
  const auto t = terminals(run(d, 0.1));
  ASSERT_EQ(t.size(), 3u);
  for (const auto& l : t) EXPECT_EQ(l, "ERR,BAD_CMD");
}

TEST(VirtualStm, ZeroLengthMoveAcksImmediately) {
  VirtualStm d(kStart);
  d.receive("MOV,F,0\n");
  const auto t = terminals(run(d, 0.05));
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0], "ACK");
}

TEST(VirtualStm, TelemetryArrivesAtAboutTenHertzWhileMoving) {
  VirtualStm d(kStart);
  d.receive("MOV,F,100\n");  // takes about 3.6 s
  const auto lines = run(d, 5.0);
  int tel = 0;
  for (const auto& l : lines) {
    const auto r = parse_reply(l);
    if (r && std::holds_alternative<Telemetry>(*r)) ++tel;
  }
  EXPECT_GE(tel, 30);
  EXPECT_LE(tel, 40);
}

TEST(VirtualStm, NoTelemetryWhileIdle) {
  VirtualStm d(kStart);
  EXPECT_TRUE(run(d, 2.0).empty());
}

TEST(VirtualStm, TelemetryHeadingMatchesTheTurnProgress) {
  VirtualStm d(kStart);
  d.receive("MOV,FL,90\n");
  const auto lines = run(d, 5.0);
  double last_heading = -1;
  for (const auto& l : lines) {
    const auto r = parse_reply(l);
    if (r && std::holds_alternative<Telemetry>(*r))
      last_heading = std::get<Telemetry>(*r).heading_deg;
  }
  EXPECT_GT(last_heading, 60.0);  // heading was well on its way to 90 deg at the last report
  EXPECT_LE(last_heading, 90.5);
}

TEST(VirtualStm, EncoderCountsFollowSeventyFiveCountsPerCm) {
  VirtualStm d(kStart);
  d.receive("MOV,F,40\n");
  run(d, 5.0);
  EXPECT_EQ(d.left_counts(), 40 * 75);
  EXPECT_EQ(d.right_counts(), 40 * 75);
}

TEST(VirtualStm, ReverseDrivesEncodersNegative) {
  VirtualStm d(kStart);
  d.receive("MOV,B,20\n");
  run(d, 5.0);
  EXPECT_EQ(d.left_counts(), -20 * 75);
}

TEST(VirtualStm, InnerWheelTravelsLessThanOuterWheelInATurn) {
  VirtualStm d(kStart);
  d.receive("MOV,FL,90\n");  // centre on the left: the left wheel is the inner one
  run(d, 5.0);
  EXPECT_LT(d.left_counts(), d.right_counts());
  EXPECT_GT(d.left_counts(), 0);
}

// ---------------------------------------------------------------- STOP

TEST(VirtualStm, StopWhileMovingReportsOnlyAckStoppedAndFreezesThePose) {
  VirtualStm d(kStart);
  d.receive("MOV,F,100\n");
  d.step(1.0);
  d.receive("STOP\n");
  const auto lines = run(d, 1.0);
  const auto t = terminals(lines);
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0], "ACK,STOPPED");  // the interrupted command gets no ACK/ERR of its own
  EXPECT_FALSE(d.moving());
  EXPECT_LT(d.true_pose().x, 150.0);  // stopped short of the 100 cm target
  const double frozen = d.true_pose().x;
  d.step(2.0);
  EXPECT_DOUBLE_EQ(d.true_pose().x, frozen);
}

TEST(VirtualStm, EndEventPrecedesAckStoppedAndSaysStopped) {
  VirtualStm d(kStart);
  d.receive("MOV,F,100\n");
  d.step(1.0);
  d.take_output();
  d.receive("STOP\n");
  const auto lines = run(d, 1.0);
  ASSERT_GE(lines.size(), 2u);
  EXPECT_EQ(lines.back(), "ACK,STOPPED");
  EXPECT_NE(lines[lines.size() - 2].find("RESULT:STOPPED"), std::string::npos);
}

TEST(VirtualStm, StopAtIdleRepliesOnceAfterAboutTwentyFiveMilliseconds) {
  VirtualStm d(kStart);
  d.receive("STOP\n");
  d.step(0.01);
  EXPECT_TRUE(d.take_output().empty());  // not yet
  const auto t = terminals(run(d, 0.1));
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0], "ACK,STOPPED");
}

TEST(VirtualStm, DuplicateStopIsCoalescedIntoOneReply) {
  VirtualStm d(kStart);
  d.receive("MOV,F,100\n");
  d.step(0.5);
  d.receive("STOP\nSTOP\nSTOP\n");
  const auto t = terminals(run(d, 1.0));
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0], "ACK,STOPPED");
}

TEST(VirtualStm, StopIsNeverBusy) {
  VirtualStm d(kStart);
  d.receive("MOV,F,100\n");
  d.step(0.3);
  d.receive("STOP\n");
  const auto lines = run(d, 1.0);
  for (const auto& l : lines) EXPECT_NE(l, "ERR,BUSY");
}

TEST(VirtualStm, MoveWhileStopIsOutstandingIsBusy) {
  VirtualStm d(kStart);
  d.receive("MOV,F,100\n");
  d.step(0.3);
  d.receive("STOP\nMOV,B,5\n");
  const auto t = terminals(run(d, 1.0));
  ASSERT_EQ(t.size(), 2u);
  EXPECT_EQ(t[0], "ERR,BUSY");
  EXPECT_EQ(t[1], "ACK,STOPPED");
}

TEST(VirtualStm, StopAfterAckedStopIsANewOperation) {
  VirtualStm d(kStart);
  d.receive("STOP\n");
  run(d, 0.2);
  d.receive("STOP\n");
  const auto t = terminals(run(d, 0.2));
  ASSERT_EQ(t.size(), 1u);
  EXPECT_EQ(t[0], "ACK,STOPPED");
}

// ---------------------------------------------------------------- faults

TEST(VirtualStmFaults, StalledMotorsHitTheFirmwareDeadline) {
  FaultConfig f;
  f.stall = true;
  VirtualStm d(kStart, f);
  d.receive("MOV,F,20\n");
  const auto t = terminals(run(d, 4.0));
  EXPECT_TRUE(t.empty());  // deadline is 5 s + 0.15 * 20 = 8 s: still waiting at 4 s
  const auto t2 = terminals(run(d, 5.0));
  ASSERT_EQ(t2.size(), 1u);
  EXPECT_EQ(t2[0], "ERR,MOTION_TIMEOUT");
  EXPECT_DOUBLE_EQ(d.true_pose().x, 50.0);  // it never moved
}

TEST(VirtualStmFaults, ImuFailureRejectsMovesBeforeAnythingMoves) {
  FaultConfig f;
  f.imu_fail = true;
  VirtualStm d(kStart, f);
  d.receive("MOV,F,20\nIMU,GET\n");
  const auto t = terminals(run(d, 0.2));
  ASSERT_EQ(t.size(), 2u);
  EXPECT_EQ(t[0], "ERR,IMU_NOT_READY");
  EXPECT_EQ(t[1], "ERR,IMU_NOT_READY");
  EXPECT_DOUBLE_EQ(d.true_pose().x, 50.0);
}

TEST(VirtualStmFaults, DroppedBytesCanLoseTheTerminalReplyButNeverInventOne) {
  FaultConfig f;
  f.drop_rate = 0.5;
  f.seed = 7;
  VirtualStm d(kStart, f);
  d.receive("MOV,F,10\n");
  const auto lines = run(d, 5.0);
  int acks = 0;
  for (const auto& l : lines) acks += (l == "ACK");
  EXPECT_LE(acks, 1);
  EXPECT_NEAR(d.true_pose().x, 60.0, 1e-6);  // the move itself still happened
}

TEST(VirtualStmFaults, WheelSlipShortensTravel) {
  FaultConfig f;
  f.slip = 0.1;
  VirtualStm d(kStart, f);
  d.receive("MOV,F,100\n");
  run(d, 6.0);
  EXPECT_NEAR(d.true_pose().x, 50.0 + 90.0, 1e-6);
}

TEST(VirtualStmFaults, ImuBiasShowsUpInTheReportedHeading) {
  FaultConfig f;
  f.imu_bias_deg = 3.0;
  VirtualStm d(kStart, f);
  d.receive("IMU,GET\n");
  const auto lines = run(d, 0.1);
  ASSERT_FALSE(lines.empty());
  EXPECT_NEAR(std::get<ImuReading>(*parse_reply(lines[0])).heading_deg, 3.0, 1e-9);
}

TEST(VirtualStmFaults, SameSeedGivesTheSameNoise) {
  FaultConfig f;
  f.encoder_noise = 0.02;
  f.seed = 5;
  VirtualStm a(kStart, f), b(kStart, f);
  a.receive("MOV,F,50\n");
  b.receive("MOV,F,50\n");
  run(a, 4.0);
  run(b, 4.0);
  EXPECT_EQ(a.left_counts(), b.left_counts());
}
