#include <gtest/gtest.h>

#include "stm32_protocol/protocol.hpp"

using namespace stm;

// ---------------------------------------------------------------- commands

TEST(Command, EncodesEveryMoveWithTheWireFormat) {
  EXPECT_EQ(encode_command(MoveCmd{mdp::Cmd::F, 30}), "MOV,F,30");
  EXPECT_EQ(encode_command(MoveCmd{mdp::Cmd::B, 5}), "MOV,B,5");
  EXPECT_EQ(encode_command(MoveCmd{mdp::Cmd::FL, 90}), "MOV,FL,90");
  EXPECT_EQ(encode_command(MoveCmd{mdp::Cmd::FR, 45}), "MOV,FR,45");
  EXPECT_EQ(encode_command(MoveCmd{mdp::Cmd::BL, 10}), "MOV,BL,10");
  EXPECT_EQ(encode_command(MoveCmd{mdp::Cmd::BR, 360}), "MOV,BR,360");
  EXPECT_EQ(encode_command(StopCmd{}), "STOP");
  EXPECT_EQ(encode_command(ImuGetCmd{}), "IMU,GET");
}

TEST(Command, ParseIsTheInverseOfEncode) {
  for (int i = 0; i < mdp::kNumCmds; ++i) {
    const MoveCmd m{static_cast<mdp::Cmd>(i), 17};
    const auto parsed = parse_command(encode_command(m));
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(std::get<MoveCmd>(*parsed), m);
  }
}

TEST(Command, StopAndImuGetParse) {
  EXPECT_TRUE(std::holds_alternative<StopCmd>(*parse_command("STOP")));
  EXPECT_TRUE(std::holds_alternative<ImuGetCmd>(*parse_command("IMU,GET")));
}

// PROTOCOL.md 4.2.1: every STOP variant is ERR,BAD_CMD.
TEST(Command, StopVariantsAreRejected) {
  for (const char* bad : {"STOP,", "STOP,1", "STOP ", " STOP", "stop", "Stop", "STOPX", "STOP\t"}) {
    EXPECT_FALSE(parse_command(bad).has_value()) << "[" << bad << "]";
  }
}

TEST(Command, MalformedMovesAreRejected) {
  for (const char* bad : {"", "MOV", "MOV,F", "MOV,F,", "MOV,F,x", "MOV,X,10", "MOV,F,10,5", "mov,F,10",
                          "MOV,F,-5", "MOV,F, 5", "MOV, F,5", "MOV,f,5", "MOV,F,5 ", "MOV,F,+5"}) {
    EXPECT_FALSE(parse_command(bad).has_value()) << "[" << bad << "]";
  }
}

TEST(Command, ValueLimitsFollowTheWireFormat) {
  EXPECT_TRUE(parse_command("MOV,F,999").has_value());
  EXPECT_FALSE(parse_command("MOV,F,1000").has_value());
  EXPECT_TRUE(parse_command("MOV,FL,360").has_value());
  EXPECT_FALSE(parse_command("MOV,FL,361").has_value());
  EXPECT_TRUE(parse_command("MOV,F,0").has_value());  // the device accepts 0; the planner never sends it
}

TEST(Command, LinesLongerThan31CharactersAreRejected) {
  EXPECT_FALSE(parse_command(std::string(32, 'A')).has_value());
  EXPECT_FALSE(parse_command("MOV,F,000000000000000000000000001").has_value());
}

// ---------------------------------------------------------------- replies

TEST(Reply, TerminalRepliesAreOnlyAckAckStoppedAndErr) {
  EXPECT_TRUE(is_terminal(*parse_reply("ACK")));
  EXPECT_TRUE(is_terminal(*parse_reply("ACK,STOPPED")));
  EXPECT_TRUE(is_terminal(*parse_reply("ERR,BUSY")));
  EXPECT_FALSE(is_terminal(*parse_reply("MM,EV:START,D:F")));
  EXPECT_FALSE(is_terminal(*parse_reply("MM,TEL,T:100,L:5,R:5,H:9000")));
  EXPECT_FALSE(is_terminal(*parse_reply("IMU,12.50")));
}

TEST(Reply, ErrCodesAreMapped) {
  EXPECT_EQ(std::get<Err>(*parse_reply("ERR,BAD_CMD")).code, ErrCode::BadCmd);
  EXPECT_EQ(std::get<Err>(*parse_reply("ERR,BUSY")).code, ErrCode::Busy);
  EXPECT_EQ(std::get<Err>(*parse_reply("ERR,IMU_NOT_READY")).code, ErrCode::ImuNotReady);
  EXPECT_EQ(std::get<Err>(*parse_reply("ERR,MOTION_TIMEOUT")).code, ErrCode::MotionTimeout);
}

TEST(Reply, UnknownErrCodeIsKeptVerbatim) {
  const auto e = std::get<Err>(*parse_reply("ERR,IR_SENSOR"));
  EXPECT_EQ(e.code, ErrCode::Unknown);
  EXPECT_EQ(e.raw, "IR_SENSOR");
  EXPECT_EQ(encode_reply(e), "ERR,IR_SENSOR");
}

TEST(Reply, AckStoppedIsNotPlainAck) {
  EXPECT_TRUE(std::holds_alternative<AckStopped>(*parse_reply("ACK,STOPPED")));
  EXPECT_TRUE(std::holds_alternative<Ack>(*parse_reply("ACK")));
}

TEST(Reply, TelemetryRoundTrips) {
  const Telemetry t{1500, -1234, 5678, 90.25};
  const auto parsed = parse_reply(encode_reply(t));
  ASSERT_TRUE(parsed.has_value());
  const auto& back = std::get<Telemetry>(*parsed);
  EXPECT_EQ(back.t_ms, 1500);
  EXPECT_EQ(back.left_counts, -1234);
  EXPECT_EQ(back.right_counts, 5678);
  EXPECT_NEAR(back.heading_deg, 90.25, 1e-9);
}

TEST(Reply, GarbageIsNotAReply) {
  for (const char* bad : {"", "HELLO", "ACKK", "ERR", "MM,TEL,T:1", "MM,TEL,T:a,L:1,R:1,H:1", "IMU,abc"}) {
    EXPECT_FALSE(parse_reply(bad).has_value()) << "[" << bad << "]";
  }
}

TEST(Reply, EventLinesAreForwardedAsTelemetryNotReplies) {
  const auto r = parse_reply("MM,EV:END,D:F,T:1800,RESULT:OK,DL:750,DR:750");
  ASSERT_TRUE(r.has_value());
  EXPECT_TRUE(std::holds_alternative<EventLine>(*r));
  EXPECT_FALSE(is_terminal(*r));
}

// ---------------------------------------------------------------- framing

TEST(LineFramer, ReassemblesALineSplitAcrossReads) {
  LineFramer f;
  EXPECT_TRUE(f.feed("MO").empty());
  EXPECT_TRUE(f.feed("V,F,").empty());
  const auto lines = f.feed("30\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "MOV,F,30");
}

TEST(LineFramer, SplitsSeveralLinesArrivingTogether) {
  LineFramer f;
  const auto lines = f.feed("ACK\nERR,BUSY\nIMU,1.00\n");
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[1], "ERR,BUSY");
}

TEST(LineFramer, AcceptsCrLfTerminators) {
  LineFramer f;
  const auto lines = f.feed("ACK\r\nSTOP\r\n");
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "ACK");
  EXPECT_EQ(lines[1], "STOP");
}

TEST(LineFramer, ByteAtATimeDeliveryGivesTheSameLines) {
  LineFramer f;
  std::vector<std::string> all;
  for (char ch : std::string("MOV,FL,90\nSTOP\n")) {
    for (auto& l : f.feed(std::string(1, ch))) all.push_back(l);
  }
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[0], "MOV,FL,90");
}

TEST(LineFramer, OverlongLineIsDiscardedAndTheFramerResynchronises) {
  LineFramer f;
  const auto lines = f.feed(std::string(100, 'X') + "\nACK\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "ACK");
  EXPECT_EQ(f.overflow_count(), 1);
}

TEST(LineFramer, MemoryStaysBoundedOnAnEndlessGarbageStream) {
  LineFramer f;
  for (int i = 0; i < 1000; ++i) f.feed(std::string(1000, 'Z'));
  const auto lines = f.feed("\nACK\n");
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_EQ(lines[0], "ACK");
}

TEST(LineFramer, EmptyLinesAreReportedAsEmptyStrings) {
  LineFramer f;
  const auto lines = f.feed("\n\n");
  EXPECT_EQ(lines.size(), 2u);
}
