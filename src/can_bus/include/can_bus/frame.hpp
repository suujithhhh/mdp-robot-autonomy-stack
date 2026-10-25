// Classical CAN 2.0A (11-bit identifier) data frames: bit-exact encoding, bit stuffing, CRC-15.
//
// On the wire a frame is, MSB first: SOF(0), ID[10:0], RTR(0), IDE(0), r0(0), DLC[3:0], data,
// CRC[14:0], CRC delimiter(1), ACK slot, ACK delimiter(1), EOF(1 x 7), then 3 bits of
// intermission. Everything from SOF to the end of the CRC is bit-stuffed: after five equal bits the
// transmitter inserts one bit of the opposite value, so receivers always see edges to
// resynchronise on and can detect a stuck line (six equal bits = stuff error).
//
// Bit values follow the bus: 0 = dominant, 1 = recessive.
//
// Not supported (documented limits of this simulator): remote frames (RTR), 29-bit extended
// identifiers, CAN FD.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace can {

constexpr std::uint16_t kMaxId = 0x7FF;
constexpr int kMaxDlc = 8;
// CRC delimiter + ACK slot + ACK delimiter + 7 EOF + 3 intermission bits after the stuffed region.
constexpr int kTrailerBits = 1 + 1 + 1 + 7 + 3;

struct Frame {
  std::uint16_t id{0};  // 11 bits; a lower value wins arbitration
  std::uint8_t dlc{0};  // number of data bytes, 0..8
  std::array<std::uint8_t, 8> data{};

  bool operator==(const Frame& o) const;
};

using Bits = std::vector<std::uint8_t>;  // one entry per bit, 0 or 1

// CRC-15/CAN: polynomial x^15+x^14+x^10+x^8+x^7+x^4+x^3+1 (0x4599), initial value 0.
std::uint16_t crc15(const Bits& bits);

// The unstuffed bits the CRC covers: SOF through the last data bit.
Bits crc_region(const Frame& frame);

// Insert/remove stuff bits. destuff() returns false on a stuff error (six equal bits in a row).
Bits stuff(const Bits& in);
bool destuff(const Bits& in, Bits& out);

// Full transmitted bit sequence (stuffed SOF..CRC, then the trailer, ACK slot left recessive).
Bits encode(const Frame& frame);

enum class DecodeError { None, Stuff, Crc, Form, Dlc, Unsupported };
const char* decode_error_name(DecodeError e);

struct DecodeResult {
  DecodeError error{DecodeError::None};
  Frame frame{};
};

// Parse the bits a receiver saw (the stuffed region plus at least the CRC delimiter).
DecodeResult decode(const Bits& wire);

}  // namespace can
