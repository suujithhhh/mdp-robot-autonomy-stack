#include <gtest/gtest.h>

#include "can_bus/robot_can.hpp"

using namespace can;

namespace {

// Device-side line -> frames -> host-side decoder -> the line the driver would have received.
std::vector<std::string> through_can(const std::string& line) {
  HostDecoder decoder;
  std::vector<std::string> out;
  for (const auto& frame : line_to_frames(line)) {
    // Also push each frame through the real bit encoding, as the bus would.
    const auto decoded = decode(encode(frame));
    EXPECT_EQ(decoded.error, DecodeError::None);
    for (auto& l : decoder.feed(decoded.frame)) out.push_back(l);
  }
  return out;
}

}  // namespace

TEST(RobotCan, EveryMoveCommandSurvivesTheTripToTheDevice) {
  for (int c = 0; c < mdp::kNumCmds; ++c) {
    for (int value : {0, 1, 17, 360, 999}) {
      const auto cmd = static_cast<mdp::Cmd>(c);
      if (value > mdp::max_value(cmd)) continue;
      const stm::Command command = stm::MoveCmd{cmd, value};
      const Frame frame = command_to_frame(command);
      EXPECT_EQ(frame.id, id::kMove);
      EXPECT_EQ(frame_to_command_line(frame), stm::encode_command(command));
    }
  }
}

TEST(RobotCan, StopAndImuGetHaveTheirOwnFrames) {
  EXPECT_EQ(command_to_frame(stm::StopCmd{}).id, id::kStop);
  EXPECT_EQ(command_to_frame(stm::ImuGetCmd{}).id, id::kImuGet);
  EXPECT_EQ(frame_to_command_line(command_to_frame(stm::StopCmd{})), "STOP");
  EXPECT_EQ(frame_to_command_line(command_to_frame(stm::ImuGetCmd{})), "IMU,GET");
  EXPECT_EQ(command_to_frame(stm::StopCmd{}).dlc, 0);
}

TEST(RobotCan, SafetyAndCommandFramesOutrankStatusAndTelemetry) {
  EXPECT_LT(id::kStop, id::kMove);
  EXPECT_LT(id::kMove, id::kReply);
  EXPECT_LT(id::kReply, id::kEventStart);
  EXPECT_LT(id::kEventStart, id::kTelemetryA);
  EXPECT_LT(id::kEventEndA, id::kTelemetryA);
}

TEST(RobotCan, MalformedCommandFramesBecomeAnUnparsableLineSoTheDeviceAnswersBadCmd) {
  Frame short_move;
  short_move.id = id::kMove;
  short_move.dlc = 2;
  EXPECT_EQ(frame_to_command_line(short_move), "BAD_CAN_FRAME");

  Frame bad_cmd = command_to_frame(stm::MoveCmd{mdp::Cmd::F, 10});
  bad_cmd.data[0] = 9;
  EXPECT_EQ(frame_to_command_line(bad_cmd), "BAD_CAN_FRAME");

  Frame too_far = command_to_frame(stm::MoveCmd{mdp::Cmd::F, 10});
  too_far.data[1] = 0xE8;  // 1000
  too_far.data[2] = 0x03;
  EXPECT_EQ(frame_to_command_line(too_far), "BAD_CAN_FRAME");

  Frame stop_with_data = command_to_frame(stm::StopCmd{});
  stop_with_data.dlc = 1;
  EXPECT_EQ(frame_to_command_line(stop_with_data), "BAD_CAN_FRAME");
  EXPECT_FALSE(stm::parse_command("BAD_CAN_FRAME").has_value());
}

TEST(RobotCan, FramesThatAreNotCommandsAreIgnoredByTheDevice) {
  Frame f;
  f.id = 0x200;
  EXPECT_FALSE(frame_to_command_line(f).has_value());
  f.id = id::kTelemetryA;
  EXPECT_FALSE(frame_to_command_line(f).has_value());
}

TEST(RobotCan, RepliesRoundTripToTheSameUartLine) {
  for (const char* line : {"ACK", "ACK,STOPPED", "ERR,BAD_CMD", "ERR,BUSY", "ERR,IMU_NOT_READY",
                           "ERR,MOTION_TIMEOUT"}) {
    const auto out = through_can(line);
    ASSERT_EQ(out.size(), 1u) << line;
    EXPECT_EQ(out[0], line);
  }
}

TEST(RobotCan, ImuReadingsKeepCentiDegreePrecisionIncludingNegatives) {
  for (const char* line : {"IMU,0.00", "IMU,12.50", "IMU,-179.99", "IMU,90.00", "IMU,3.07"}) {
    const auto out = through_can(line);
    ASSERT_EQ(out.size(), 1u) << line;
    EXPECT_EQ(out[0], line);
  }
}

TEST(RobotCan, TelemetryTakesTwoFramesAndComesBackIdentical) {
  const std::string line = stm::encode_reply(stm::Telemetry{1500, -1234, 56789, 90.25});
  EXPECT_EQ(line_to_frames(line).size(), 2u);
  const auto out = through_can(line);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], line);
}

TEST(RobotCan, LargeEncoderCountsFitTheThirtyTwoBitFields) {
  const std::string line = stm::encode_reply(stm::Telemetry{120000, 74925, -74925, -45.5});
  const auto out = through_can(line);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], line);
}

TEST(RobotCan, StartAndEndEventsRoundTrip) {
  for (const char* line :
       {"MM,EV:START,D:F", "MM,EV:START,D:BL", "MM,EV:END,D:FL,T:2530,RESULT:OK,DL:1234,DR:1500",
        "MM,EV:END,D:B,T:900,RESULT:STOPPED,DL:-300,DR:-310",
        "MM,EV:END,D:F,T:8100,RESULT:TIMEOUT,DL:0,DR:0"}) {
    const auto out = through_can(line);
    ASSERT_EQ(out.size(), 1u) << line;
    EXPECT_EQ(out[0], line);
  }
}

TEST(RobotCan, TheSecondHalfOfAMessageWithoutItsFirstHalfIsDroppedAndCounted) {
  HostDecoder decoder;
  const auto frames = line_to_frames(stm::encode_reply(stm::Telemetry{100, 1, 2, 3.0}));
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_TRUE(decoder.feed(frames[1]).empty());  // B without A
  EXPECT_EQ(decoder.dropped(), 1);
  EXPECT_TRUE(decoder.feed(frames[0]).empty());   // A alone produces nothing yet
  EXPECT_EQ(decoder.feed(frames[1]).size(), 1u);  // B completes the pair
}

TEST(RobotCan, AStaleFirstHalfIsReplacedByTheNewestOne) {
  HostDecoder decoder;
  const auto old_frames = line_to_frames(stm::encode_reply(stm::Telemetry{100, 1, 2, 3.0}));
  const auto new_frames = line_to_frames(stm::encode_reply(stm::Telemetry{200, 50, 60, 4.0}));
  decoder.feed(old_frames[0]);  // A(old) then the old B is lost
  decoder.feed(new_frames[0]);  // A(new)
  const auto out = decoder.feed(new_frames[1]);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], stm::encode_reply(stm::Telemetry{200, 50, 60, 4.0}));
}

TEST(RobotCan, UnknownIdentifiersAreCountedNotCrashed) {
  HostDecoder decoder;
  Frame f;
  f.id = 0x555;
  f.dlc = 1;
  EXPECT_TRUE(decoder.feed(f).empty());
  EXPECT_EQ(decoder.dropped(), 1);
}

TEST(RobotCan, LinesWithoutACanRepresentationProduceNoFrames) {
  EXPECT_TRUE(line_to_frames("").empty());
  EXPECT_TRUE(line_to_frames("garbage").empty());
  EXPECT_TRUE(line_to_frames("MM,EV:START,D:ZZ").empty());
}

TEST(RobotCan, EveryFrameFitsInEightDataBytes) {
  for (const char* line :
       {"ACK", "IMU,12.50", "MM,EV:START,D:F", "MM,EV:END,D:FL,T:2530,RESULT:OK,DL:1234,DR:1500"}) {
    for (const auto& f : line_to_frames(line)) EXPECT_LE(f.dlc, 8);
  }
  for (const auto& f : line_to_frames(stm::encode_reply(stm::Telemetry{1, 2, 3, 4.0})))
    EXPECT_LE(f.dlc, 8);
}
