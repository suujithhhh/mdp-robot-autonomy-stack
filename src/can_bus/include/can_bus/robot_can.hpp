// How the robot's UART protocol maps onto CAN messages.
//
// CAN has no "lines": each message is an identifier plus up to 8 data bytes, and the identifier
// decides priority (lower wins arbitration). The mapping below keeps the meaning of every UART
// message and gives safety-critical traffic the lowest identifiers:
//
// clang-format off
//   id     direction     meaning                data bytes (little endian)
//   0x001  host->device  STOP (emergency stop)  -
//   0x010  host->device  MOVE                   [cmd u8][value u16]
//   0x011  host->device  IMU_GET                -
//   0x080  device->host  REPLY                  [type u8: 0 ACK, 1 ACK_STOPPED, 2 ERR][err code u8]
//   0x081  device->host  IMU_READING            [centi-degrees i16]
//   0x0F0  device->host  EVENT_START            [cmd u8]
//   0x0F1  device->host  EVENT_END_A            [cmd u8][result u8][duration ms u32]
//   0x0F2  device->host  EVENT_END_B            [DL counts i32][DR counts i32]
//   0x100  device->host  TELEMETRY_A            [left counts i32][right counts i32]
//   0x101  device->host  TELEMETRY_B            [t ms u32][heading centi-degrees i16]
// clang-format on
//
// Two-frame messages (END, TELEMETRY) are rebuilt by HostDecoder, which hands the rest of the
// stack the same text lines the UART link would have produced, so the driver code is unchanged.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "can_bus/frame.hpp"
#include "stm32_protocol/protocol.hpp"

namespace can {

namespace id {
constexpr std::uint16_t kStop = 0x001;
constexpr std::uint16_t kMove = 0x010;
constexpr std::uint16_t kImuGet = 0x011;
constexpr std::uint16_t kReply = 0x080;
constexpr std::uint16_t kImuReading = 0x081;
constexpr std::uint16_t kEventStart = 0x0F0;
constexpr std::uint16_t kEventEndA = 0x0F1;
constexpr std::uint16_t kEventEndB = 0x0F2;
constexpr std::uint16_t kTelemetryA = 0x100;
constexpr std::uint16_t kTelemetryB = 0x101;
}  // namespace id

// ---- host -> device ------------------------------------------------------------------

Frame command_to_frame(const stm::Command& command);

// Device side: the UART-style command line for a command frame ("MOV,FL,90", "STOP",
// "IMU,GET"). A malformed command frame yields a line the device answers with ERR,BAD_CMD.
// Frames that are not commands yield nullopt.
std::optional<std::string> frame_to_command_line(const Frame& frame);

// ---- device -> host ------------------------------------------------------------------

// Device side: translate one UART-style output line into CAN frames (empty if the line has no
// CAN representation).
std::vector<Frame> line_to_frames(const std::string& line);

// Host side: turns incoming frames back into UART-style lines, pairing the two-frame messages.
class HostDecoder {
 public:
  std::vector<std::string> feed(const Frame& frame);
  // Second halves that arrived without their first half, and unknown identifiers.
  long dropped() const { return dropped_; }

 private:
  struct PendingTelemetry {
    std::int32_t left;
    std::int32_t right;
  };
  struct PendingEnd {
    std::uint8_t cmd;
    std::uint8_t result;
    std::uint32_t t_ms;
  };
  std::optional<PendingTelemetry> telemetry_;
  std::optional<PendingEnd> end_;
  long dropped_{0};
};

}  // namespace can
