// UART protocol shared by the ROS 2 driver (host side) and the virtual STM32 (device side).
// Pure C++, no ROS and no OS dependencies. Follows PROTOCOL.md section 4 of the MDP project:
//   * ASCII lines, '\n' terminated ('\r\n' accepted), at most 31 characters per line
//   * exactly one motion command in flight
//   * terminal replies: "ACK", "ERR,<code>", and "ACK,STOPPED" (the reply to STOP)
//   * every other line (MM,...) is telemetry and is never a reply
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "mdp_planner_core/primitives.hpp"

namespace stm {

// Commands sent to the device are at most 31 characters (the STM's line buffer). Lines coming
// back (telemetry, MM,EV:END,...) are longer, so the host-side framer uses a larger limit.
constexpr std::size_t kMaxLineChars = 31;
constexpr std::size_t kMaxHostLineChars = 160;

// ---- host -> device -------------------------------------------------------------------

struct MoveCmd {
  mdp::Cmd cmd;
  int value;  // cm for F/B, degrees for turns
  bool operator==(const MoveCmd& o) const { return cmd == o.cmd && value == o.value; }
};
struct StopCmd {
  bool operator==(const StopCmd&) const { return true; }
};
struct ImuGetCmd {
  bool operator==(const ImuGetCmd&) const { return true; }
};
using Command = std::variant<MoveCmd, StopCmd, ImuGetCmd>;

// "MOV,FL,90", "STOP", "IMU,GET" (without the terminator).
std::string encode_command(const Command& c);
// Strict: any deviation (spaces, lowercase, trailing junk, out-of-range value) yields nullopt,
// which the device answers with ERR,BAD_CMD.
std::optional<Command> parse_command(std::string_view line);

// ---- device -> host -------------------------------------------------------------------

enum class ErrCode { BadCmd, Busy, ImuNotReady, MotionTimeout, Unknown };
const char* err_code_name(ErrCode e);

struct Ack {};
struct AckStopped {};
struct Err {
  ErrCode code;
  std::string raw;  // the text after "ERR,", kept for unknown codes
};
struct ImuReading {
  double heading_deg;
};
// Telemetry: "MM,TEL,T:<ms>,L:<left counts>,R:<right counts>,H:<heading centi-degrees>"
struct Telemetry {
  long t_ms;
  long left_counts;
  long right_counts;
  double heading_deg;
};
struct EventLine {
  std::string text;  // MM,EV:START / MM,EV:END lines, forwarded for logging only
};
using Reply = std::variant<Ack, AckStopped, Err, ImuReading, Telemetry, EventLine>;

std::string encode_reply(const Reply& r);
std::optional<Reply> parse_reply(std::string_view line);

// True for the replies that terminate a motion command.
bool is_terminal(const Reply& r);

// ---- framing --------------------------------------------------------------------------

// Rebuilds complete lines from arbitrary chunks of bytes (a serial read may return half a line,
// or three lines at once). An over-long line is discarded and counted; the framer resynchronises
// at the next newline instead of growing without bound.
class LineFramer {
 public:
  explicit LineFramer(std::size_t max_chars = kMaxLineChars) : max_chars_(max_chars) {}
  std::vector<std::string> feed(std::string_view bytes);
  long overflow_count() const { return overflows_; }

 private:
  std::size_t max_chars_;
  std::string buffer_;
  bool discarding_{false};
  long overflows_{0};
};

}  // namespace stm
