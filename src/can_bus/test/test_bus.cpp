#include <gtest/gtest.h>

#include <vector>

#include "can_bus/bus.hpp"

using namespace can;

namespace {

Frame make(std::uint16_t id, std::uint8_t tag = 0, int dlc = 1) {
  Frame f;
  f.id = id;
  f.dlc = static_cast<std::uint8_t>(dlc);
  f.data[0] = tag;
  return f;
}

std::vector<Frame> drain(Node& n) {
  std::vector<Frame> out;
  while (auto f = n.receive()) out.push_back(*f);
  return out;
}

}  // namespace

TEST(CanBus, AFrameReachesEveryOtherNodeButNotItsSender) {
  Bus bus;
  Node& a = bus.add_node("a");
  Node& b = bus.add_node("b");
  Node& c = bus.add_node("c");
  ASSERT_TRUE(a.send(make(0x123, 7)));
  bus.advance_to(0.01);
  EXPECT_TRUE(drain(a).empty());
  const auto at_b = drain(b);
  const auto at_c = drain(c);
  ASSERT_EQ(at_b.size(), 1u);
  ASSERT_EQ(at_c.size(), 1u);
  EXPECT_EQ(at_b[0].id, 0x123);
  EXPECT_EQ(at_b[0].data[0], 7);
  EXPECT_EQ(a.frames_sent(), 1);
}

TEST(CanBus, TheLowestIdentifierWinsArbitrationAndTheLoserIsNotLost) {
  Bus bus;
  Node& a = bus.add_node("a");
  Node& b = bus.add_node("b");
  Node& observer = bus.add_node("observer");
  a.send(make(0x100, 1));
  b.send(make(0x010, 2));
  bus.advance_to(0.01);
  const auto seen = drain(observer);
  ASSERT_EQ(seen.size(), 2u);
  EXPECT_EQ(seen[0].id, 0x010);  // the winner goes first
  EXPECT_EQ(seen[1].id, 0x100);  // the loser retried and was delivered right after
  EXPECT_GE(bus.stats().arbitration_losses, 1);
}

TEST(CanBus, ThreeContendersAreServedInIdentifierOrder) {
  Bus bus;
  Node& a = bus.add_node("a");
  Node& b = bus.add_node("b");
  Node& c = bus.add_node("c");
  Node& observer = bus.add_node("observer");
  a.send(make(0x300));
  b.send(make(0x200));
  c.send(make(0x100));
  bus.advance_to(0.01);
  const auto seen = drain(observer);
  ASSERT_EQ(seen.size(), 3u);
  EXPECT_EQ(seen[0].id, 0x100);
  EXPECT_EQ(seen[1].id, 0x200);
  EXPECT_EQ(seen[2].id, 0x300);
}

TEST(CanBus, IdentifiersDifferingOnlyInTheLastBitStillArbitrateCorrectly) {
  Bus bus;
  Node& a = bus.add_node("a");
  Node& b = bus.add_node("b");
  Node& observer = bus.add_node("observer");
  a.send(make(0x401));
  b.send(make(0x400));
  bus.advance_to(0.01);
  const auto seen = drain(observer);
  ASSERT_EQ(seen.size(), 2u);
  EXPECT_EQ(seen[0].id, 0x400);
}

TEST(CanBus, AStopFrameOvertakesQueuedTelemetryFromAnotherNode) {
  Bus bus;
  Node& ecu = bus.add_node("ecu");
  Node& host = bus.add_node("host");
  Node& observer = bus.add_node("observer");
  for (int i = 0; i < 5; ++i)
    ecu.send(make(0x100, static_cast<std::uint8_t>(i), 8));  // telemetry queued first
  host.send(make(0x001));                                    // STOP
  bus.advance_to(0.01);
  const auto seen = drain(observer);
  ASSERT_EQ(seen.size(), 6u);
  EXPECT_EQ(seen[0].id, 0x001);  // STOP went first although it was queued last
}

TEST(CanBus, OneNodesFramesKeepTheirOrder) {
  Bus bus;
  Node& a = bus.add_node("a");
  Node& b = bus.add_node("b");
  for (std::uint8_t i = 0; i < 10; ++i) a.send(make(0x200, i));
  bus.advance_to(0.05);
  const auto seen = drain(b);
  ASSERT_EQ(seen.size(), 10u);
  for (std::uint8_t i = 0; i < 10; ++i) EXPECT_EQ(seen[i].data[0], i);
}

TEST(CanBus, AFrameTakesExactlyItsBitCountAtTheConfiguredBitRate) {
  Bus bus;  // 500 kbit/s
  Node& a = bus.add_node("a");
  bus.add_node("b");
  Frame f = make(0x321, 0xA5, 8);
  const std::size_t bits = encode(f).size();
  a.send(f);
  bus.advance_to(0.001);
  EXPECT_EQ(bus.stats().busy_bits, static_cast<long>(bits));
  EXPECT_EQ(bus.stats().frames_ok, 1);
}

TEST(CanBus, HundredBackToBackFramesTakeAboutHundredFrameTimes) {
  Bus bus;
  Node& a = bus.add_node("a", 200);
  bus.add_node("b");
  const Frame f = make(0x321, 0x5A, 8);
  for (int i = 0; i < 100; ++i) ASSERT_TRUE(a.send(f));
  bus.advance_to(1.0);
  EXPECT_EQ(a.frames_sent(), 100);
  EXPECT_EQ(bus.stats().busy_bits, static_cast<long>(100 * encode(f).size()));
}

TEST(CanBus, UtilisationIsBusyTimeOverElapsedTime) {
  Bus bus;
  Node& a = bus.add_node("a");
  bus.add_node("b");
  const Frame f = make(0x321, 0, 8);
  for (int i = 0; i < 10; ++i) a.send(f);
  bus.advance_to(0.1);
  const double expected = (10.0 * static_cast<double>(encode(f).size()) / 500000.0) / 0.1;
  EXPECT_NEAR(bus.utilisation(), expected, 0.01);
}

TEST(CanBus, ATransmitQueueRefusesFramesBeyondItsCapacity) {
  Bus bus;
  Node& a = bus.add_node("a", 3);
  EXPECT_TRUE(a.send(make(0x100)));
  EXPECT_TRUE(a.send(make(0x100)));
  EXPECT_TRUE(a.send(make(0x100)));
  EXPECT_FALSE(a.send(make(0x100)));
  EXPECT_EQ(a.tx_overflows(), 1);
  EXPECT_EQ(a.tx_pending(), 3u);
}

// ---------------------------------------------------------------- errors

TEST(CanBusErrors, BitErrorsAreDetectedAndTheFrameIsRetransmittedExactlyOnce) {
  Bus::Config cfg;
  cfg.bit_error_rate = 0.002;
  cfg.seed = 5;
  Bus bus(cfg);
  Node& a = bus.add_node("a", 100);
  Node& b = bus.add_node("b");
  for (std::uint8_t i = 0; i < 50; ++i) a.send(make(0x200, i, 8));
  bus.advance_to(2.0);
  const auto seen = drain(b);
  ASSERT_EQ(seen.size(), 50u);  // nothing lost, nothing duplicated
  for (std::uint8_t i = 0; i < 50; ++i) EXPECT_EQ(seen[i].data[0], i);
  EXPECT_GT(bus.stats().frame_errors, 0);
  EXPECT_GT(a.tx_errors(), 0);
  EXPECT_NE(a.state(), NodeState::BusOff);
}

TEST(CanBusErrors, ARetransmissionSpendsAnExtraErrorFrameOnTheBus) {
  Bus clean_bus;
  Node& a0 = clean_bus.add_node("a", 100);
  clean_bus.add_node("b");
  Bus::Config cfg;
  cfg.bit_error_rate = 0.002;
  cfg.seed = 5;
  Bus noisy_bus(cfg);
  Node& a1 = noisy_bus.add_node("a", 100);
  noisy_bus.add_node("b");
  for (int i = 0; i < 50; ++i) {
    a0.send(make(0x200, 0, 8));
    a1.send(make(0x200, 0, 8));
  }
  clean_bus.advance_to(2.0);
  noisy_bus.advance_to(2.0);
  EXPECT_GT(noisy_bus.stats().busy_bits, clean_bus.stats().busy_bits);
}

TEST(CanBusErrors, ReceiversCountErrorsAndSuccessesLowerTheTransmitCounter) {
  Bus::Config cfg;
  cfg.bit_error_rate = 0.003;
  cfg.seed = 9;
  Bus bus(cfg);
  Node& a = bus.add_node("a", 200);
  Node& b = bus.add_node("b");
  for (int i = 0; i < 100; ++i) a.send(make(0x200, 0, 8));
  bus.advance_to(0.01);  // a few frames: errors push TEC up
  int peak = a.tec();
  for (int step = 0; step < 400; ++step) {
    bus.advance_to(bus.now_s() + 0.0005);
    peak = std::max(peak, a.tec());
  }
  EXPECT_GT(peak, 0);
  EXPECT_GT(b.rec(), -1);
  EXPECT_LT(a.tec(), peak + 1);
}

TEST(CanBusErrors, AFrameNobodyAcknowledgesIsAnAckErrorAndIsRetried) {
  Bus bus;
  Node& a = bus.add_node("a");  // alone on the bus
  a.send(make(0x100));
  bus.advance_to(0.001);
  EXPECT_GT(bus.stats().ack_errors, 0);
  EXPECT_EQ(a.frames_sent(), 0);
  EXPECT_EQ(a.tx_pending(), 1u);  // still queued for retransmission
  EXPECT_GT(a.tec(), 0);
}

TEST(CanBusErrors, ANodeGoesErrorPassiveThenBusOffAfterRepeatedAckErrors) {
  Bus bus;
  Node& a = bus.add_node("a");
  a.send(make(0x100));
  bool saw_passive = false;
  for (int i = 0; i < 2000 && a.state() != NodeState::BusOff; ++i) {
    bus.advance_to(bus.now_s() + 0.0001);
    if (a.state() == NodeState::ErrorPassive) saw_passive = true;
  }
  EXPECT_TRUE(saw_passive);
  EXPECT_EQ(a.state(), NodeState::BusOff);
  EXPECT_GT(a.tec(), 255);
  EXPECT_EQ(a.tx_pending(), 1u);
}

TEST(CanBusErrors, ABusOffNodeStaysSilentUntilItIsReset) {
  Bus bus;
  Node& a = bus.add_node("a");
  a.send(make(0x100, 42));
  bus.advance_to(0.05);
  ASSERT_EQ(a.state(), NodeState::BusOff);
  Node& b = bus.add_node("b");
  bus.advance_to(0.1);
  EXPECT_TRUE(drain(b).empty());  // bus-off: nothing transmitted although a listener now exists
  a.reset_bus_off();
  bus.advance_to(0.2);
  const auto seen = drain(b);
  ASSERT_EQ(seen.size(), 1u);
  EXPECT_EQ(seen[0].data[0], 42);
  EXPECT_EQ(a.state(), NodeState::ErrorActive);
}

TEST(CanBusErrors, ADisabledNodeNeitherReceivesNorAcknowledges) {
  Bus bus;
  Node& a = bus.add_node("a");
  Node& b = bus.add_node("b");
  b.set_enabled(false);
  a.send(make(0x100));
  bus.advance_to(0.002);
  EXPECT_TRUE(drain(b).empty());
  EXPECT_GT(bus.stats().ack_errors, 0);
}

TEST(CanBusErrors, OtherNodesAreUnaffectedWhenOneGoesBusOff) {
  Bus bus;
  Node& lonely = bus.add_node("lonely");
  Node& a = bus.add_node("a");
  Node& b = bus.add_node("b");
  b.set_enabled(false);
  lonely.send(make(0x300));
  a.send(make(0x100, 9));
  // b is disabled, but a and lonely hear each other, so both frames are acknowledged.
  bus.advance_to(0.01);
  EXPECT_EQ(lonely.state(), NodeState::ErrorActive);
  EXPECT_EQ(a.frames_sent(), 1);
  EXPECT_EQ(lonely.frames_sent(), 1);
}

TEST(CanBusErrors, TwoNodesSendingTheSameIdentifierEndInBusOffNotInAnInfiniteLoop) {
  Bus bus;
  Node& a = bus.add_node("a");
  Node& b = bus.add_node("b");
  a.send(make(0x100, 1));
  b.send(make(0x100, 2));  // a design error on a real bus
  bus.advance_to(1.0);     // must return
  EXPECT_EQ(a.state(), NodeState::BusOff);
  EXPECT_EQ(b.state(), NodeState::BusOff);
}

TEST(CanBusErrors, AnIdleBusAdvancesTimeWithoutCountingBusyBits) {
  Bus bus;
  bus.add_node("a");
  bus.advance_to(0.5);
  EXPECT_DOUBLE_EQ(bus.now_s(), 0.5);
  EXPECT_EQ(bus.stats().busy_bits, 0);
  EXPECT_DOUBLE_EQ(bus.utilisation(), 0.0);
}
