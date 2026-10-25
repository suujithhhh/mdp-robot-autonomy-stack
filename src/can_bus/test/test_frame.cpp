#include <gtest/gtest.h>

#include <random>

#include "can_bus/frame.hpp"

using namespace can;

namespace {

Frame make_frame(std::uint16_t id, int dlc, std::mt19937& rng) {
  Frame f;
  f.id = id;
  f.dlc = static_cast<std::uint8_t>(dlc);
  for (int i = 0; i < dlc; ++i) f.data[i] = static_cast<std::uint8_t>(rng() & 0xFF);
  return f;
}

Bits bits_of(std::initializer_list<int> v) {
  Bits b;
  for (int x : v) b.push_back(static_cast<std::uint8_t>(x));
  return b;
}

bool has_run_of_six(const Bits& b) {
  int run = 0;
  int last = -1;
  for (std::uint8_t x : b) {
    run = (x == last) ? run + 1 : 1;
    last = x;
    if (run >= 6) return true;
  }
  return false;
}

}  // namespace

// ---------------------------------------------------------------- CRC-15

// External reference value: the CRC catalogue's check value for CRC-15/CAN over "123456789".
TEST(CanCrc, MatchesTheCatalogueCheckValue) {
  Bits bits;
  for (char ch : std::string("123456789")) {
    for (int i = 7; i >= 0; --i) bits.push_back(static_cast<std::uint8_t>((ch >> i) & 1));
  }
  EXPECT_EQ(crc15(bits), 0x059E);
}

TEST(CanCrc, EmptyInputHasZeroCrc) { EXPECT_EQ(crc15(Bits{}), 0); }

TEST(CanCrc, AppendingTheCrcGivesAZeroRemainder) {
  std::mt19937 rng(1);
  for (int i = 0; i < 200; ++i) {
    Bits region = crc_region(make_frame(rng() & 0x7FF, rng() % 9, rng));
    const std::uint16_t crc = crc15(region);
    for (int b = 14; b >= 0; --b) region.push_back((crc >> b) & 1);
    EXPECT_EQ(crc15(region), 0);
  }
}

TEST(CanCrc, EverySingleBitFlipChangesTheCrc) {
  std::mt19937 rng(2);
  const Bits region = crc_region(make_frame(0x123, 8, rng));
  const std::uint16_t good = crc15(region);
  for (std::size_t i = 0; i < region.size(); ++i) {
    Bits bad = region;
    bad[i] ^= 1;
    EXPECT_NE(crc15(bad), good) << "bit " << i;
  }
}

// ---------------------------------------------------------------- stuffing

TEST(CanStuffing, FiveZerosGetAOneInserted) {
  EXPECT_EQ(stuff(bits_of({0, 0, 0, 0, 0})), bits_of({0, 0, 0, 0, 0, 1}));
}

TEST(CanStuffing, FiveOnesGetAZeroInserted) {
  EXPECT_EQ(stuff(bits_of({1, 1, 1, 1, 1})), bits_of({1, 1, 1, 1, 1, 0}));
}

TEST(CanStuffing, FourEqualBitsAreLeftAlone) {
  EXPECT_EQ(stuff(bits_of({0, 0, 0, 0, 1, 1, 1, 1})), bits_of({0, 0, 0, 0, 1, 1, 1, 1}));
}

TEST(CanStuffing, ATenZeroRunGetsTwoStuffBits) {
  EXPECT_EQ(stuff(bits_of({0, 0, 0, 0, 0, 0, 0, 0, 0, 0})),
            bits_of({0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1}));
}

TEST(CanStuffing, AStuffBitCountsTowardTheNextRun) {
  // 11111 -> stuff 0; that stuff bit plus four more zeros makes five zeros -> stuff 1.
  EXPECT_EQ(stuff(bits_of({1, 1, 1, 1, 1, 0, 0, 0, 0})),
            bits_of({1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1}));
}

TEST(CanStuffing, DestuffInvertsStuff) {
  std::mt19937 rng(3);
  for (int i = 0; i < 300; ++i) {
    Bits in;
    const int n = 1 + rng() % 120;
    for (int k = 0; k < n; ++k)
      in.push_back((rng() % 3 == 0) ? (rng() & 1) : (k > 0 ? in.back() : 0));
    Bits out;
    ASSERT_TRUE(destuff(stuff(in), out));
    EXPECT_EQ(out, in);
  }
}

TEST(CanStuffing, StuffedOutputNeverHasSixEqualBitsInARow) {
  std::mt19937 rng(4);
  for (int i = 0; i < 300; ++i) {
    Bits in(1 + rng() % 150, 0);
    for (auto& b : in) b = (rng() % 4 == 0) ? 1 : 0;  // long dominant runs on purpose
    EXPECT_FALSE(has_run_of_six(stuff(in)));
  }
}

TEST(CanStuffing, SixEqualBitsAreAStuffError) {
  Bits out;
  EXPECT_FALSE(destuff(bits_of({0, 0, 0, 0, 0, 0}), out));
  EXPECT_FALSE(destuff(bits_of({1, 1, 1, 1, 1, 1}), out));
}

// ---------------------------------------------------------------- frames

TEST(CanFrame, EncodeThenDecodeReturnsTheSameFrameForEveryDlc) {
  std::mt19937 rng(5);
  for (int dlc = 0; dlc <= 8; ++dlc) {
    for (int rep = 0; rep < 40; ++rep) {
      const Frame f = make_frame(rng() & 0x7FF, dlc, rng);
      const DecodeResult r = decode(encode(f));
      ASSERT_EQ(r.error, DecodeError::None) << "dlc " << dlc << " id " << f.id;
      EXPECT_EQ(r.frame, f);
    }
  }
}

TEST(CanFrame, ExtremeIdentifiersRoundTrip) {
  for (std::uint16_t id : {std::uint16_t{0}, std::uint16_t{1}, std::uint16_t{0x7FF},
                           std::uint16_t{0x555}, std::uint16_t{0x2AA}}) {
    Frame f;
    f.id = id;
    f.dlc = 8;
    for (int i = 0; i < 8; ++i) f.data[i] = (i % 2) ? 0xFF : 0x00;  // worst-case stuffing
    const DecodeResult r = decode(encode(f));
    ASSERT_EQ(r.error, DecodeError::None);
    EXPECT_EQ(r.frame, f);
  }
}

TEST(CanFrame, AllZeroFrameIsStuffedAndStillDecodes) {
  Frame f;  // id 0, dlc 0: lots of dominant bits in a row
  const Bits wire = encode(f);
  EXPECT_FALSE(has_run_of_six(Bits(wire.begin(), wire.end() - kTrailerBits)));
  EXPECT_EQ(decode(wire).error, DecodeError::None);
}

TEST(CanFrame, TheLongestFrameStaysWithinTheWorstCaseLength) {
  // 1 SOF + 11 ID + 3 control + 4 DLC + 64 data + 15 CRC = 98 bits; at most 1 stuff bit per
  // 4 bits after the first -> 24, plus the 13-bit trailer.
  std::mt19937 rng(6);
  std::size_t longest = 0;
  for (int i = 0; i < 2000; ++i)
    longest = std::max(longest, encode(make_frame(rng() & 0x7FF, 8, rng)).size());
  EXPECT_LE(longest, 98u + 24u + kTrailerBits);
  EXPECT_GE(longest, 98u + kTrailerBits);
}

TEST(CanFrame, EverySingleBitErrorInTheStuffedRegionIsDetected) {
  std::mt19937 rng(7);
  for (int rep = 0; rep < 60; ++rep) {
    const Frame f = make_frame(rng() & 0x7FF, rng() % 9, rng);
    const Bits wire = encode(f);
    const std::size_t stuffed_len = wire.size() - kTrailerBits;
    for (std::size_t i = 0; i < stuffed_len; ++i) {
      Bits bad = wire;
      bad[i] ^= 1;
      const DecodeResult r = decode(bad);
      EXPECT_NE(r.error, DecodeError::None)
          << "id " << f.id << " dlc " << int(f.dlc) << " bit " << i;
    }
  }
}

TEST(CanFrame, ACorruptedDataBitIsACrcOrStuffError) {
  std::mt19937 rng(8);
  const Frame f = make_frame(0x155, 8, rng);
  Bits wire = encode(f);
  wire[40] ^= 1;
  const DecodeError e = decode(wire).error;
  EXPECT_TRUE(e == DecodeError::Crc || e == DecodeError::Stuff || e == DecodeError::Form);
}

TEST(CanFrame, ATruncatedFrameIsAFormError) {
  std::mt19937 rng(9);
  Bits wire = encode(make_frame(0x100, 4, rng));
  wire.resize(30);
  EXPECT_EQ(decode(wire).error, DecodeError::Form);
  EXPECT_EQ(decode(Bits{}).error, DecodeError::Form);
}

TEST(CanFrame, ARecessiveStartOfFrameIsAFormError) {
  std::mt19937 rng(10);
  Bits wire = encode(make_frame(0x100, 1, rng));
  wire[0] = 1;
  EXPECT_NE(decode(wire).error, DecodeError::None);
}

TEST(CanFrame, DlcAboveEightIsRejected) {
  Frame f;
  f.id = 0x100;
  Bits region = crc_region(f);
  region[15] = 1;  // DLC bits are 15..18: set 1001 = 9
  region[16] = 0;
  region[17] = 0;
  region[18] = 1;
  Bits wire = stuff(region);
  for (int i = 0; i < 40; ++i) wire.push_back(1);
  EXPECT_EQ(decode(wire).error, DecodeError::Dlc);
}

TEST(CanFrame, RemoteFramesAreReportedAsUnsupported) {
  Frame f;
  f.id = 0x100;
  Bits region = crc_region(f);
  region[12] = 1;  // RTR
  const std::uint16_t crc = crc15(region);
  for (int b = 14; b >= 0; --b) region.push_back((crc >> b) & 1);
  Bits wire = stuff(region);
  for (int i = 0; i < kTrailerBits; ++i) wire.push_back(1);
  EXPECT_EQ(decode(wire).error, DecodeError::Unsupported);
}

TEST(CanFrame, DifferentIdsNeverDecodeToEachOther) {
  std::mt19937 rng(11);
  Frame a = make_frame(0x100, 2, rng), b = a;
  b.id = 0x101;
  EXPECT_NE(encode(a), encode(b));
  EXPECT_EQ(decode(encode(b)).frame.id, 0x101);
}

TEST(CanFrame, FrameEqualityIgnoresBytesBeyondTheDlc) {
  Frame a, b;
  a.dlc = b.dlc = 2;
  a.data[0] = b.data[0] = 1;
  a.data[1] = b.data[1] = 2;
  a.data[5] = 99;  // beyond dlc
  EXPECT_TRUE(a == b);
  b.data[1] = 3;
  EXPECT_FALSE(a == b);
}
