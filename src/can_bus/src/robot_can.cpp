#include "can_bus/robot_can.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace can {

namespace {

void put_u16(Frame& f, int at, std::uint16_t v) {
  f.data[at] = static_cast<std::uint8_t>(v & 0xFF);
  f.data[at + 1] = static_cast<std::uint8_t>(v >> 8);
}
void put_u32(Frame& f, int at, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) f.data[at + i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
}
std::uint16_t get_u16(const Frame& f, int at) {
  return static_cast<std::uint16_t>(f.data[at] | (f.data[at + 1] << 8));
}
std::uint32_t get_u32(const Frame& f, int at) {
  std::uint32_t v = 0;
  for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(f.data[at + i]) << (8 * i);
  return v;
}
std::int16_t clamp_i16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<std::int16_t>(v);
}

enum ReplyType : std::uint8_t { kAck = 0, kAckStopped = 1, kErr = 2 };
enum ResultCode : std::uint8_t { kOk = 0, kTimeout = 1, kStopped = 2, kOtherResult = 255 };

std::uint8_t err_to_code(stm::ErrCode c) {
  switch (c) {
    case stm::ErrCode::BadCmd:
      return 0;
    case stm::ErrCode::Busy:
      return 1;
    case stm::ErrCode::ImuNotReady:
      return 2;
    case stm::ErrCode::MotionTimeout:
      return 3;
    case stm::ErrCode::Unknown:
      return 255;
  }
  return 255;
}
std::string code_to_err_line(std::uint8_t code) {
  switch (code) {
    case 0:
      return "ERR,BAD_CMD";
    case 1:
      return "ERR,BUSY";
    case 2:
      return "ERR,IMU_NOT_READY";
    case 3:
      return "ERR,MOTION_TIMEOUT";
    default:
      return "ERR,UNKNOWN";
  }
}

std::optional<std::uint8_t> cmd_from_name(const std::string& name) {
  for (int i = 0; i < mdp::kNumCmds; ++i) {
    if (name == mdp::cmd_name(static_cast<mdp::Cmd>(i))) return static_cast<std::uint8_t>(i);
  }
  return std::nullopt;
}

// "MM,EV:END,D:F,T:1800,RESULT:OK,DL:750,DR:750" -> the text after "<key>:" up to the next comma.
std::optional<std::string> field_text(const std::string& line, const std::string& key) {
  const std::string needle = key + ":";
  std::size_t at = 0;
  while ((at = line.find(needle, at)) != std::string::npos) {
    if (at == 0 || line[at - 1] == ',') {
      const std::size_t start = at + needle.size();
      const std::size_t end = line.find(',', start);
      return line.substr(start, end == std::string::npos ? std::string::npos : end - start);
    }
    ++at;
  }
  return std::nullopt;
}

}  // namespace

Frame command_to_frame(const stm::Command& command) {
  Frame f;
  if (const auto* m = std::get_if<stm::MoveCmd>(&command)) {
    f.id = id::kMove;
    f.dlc = 3;
    f.data[0] = static_cast<std::uint8_t>(m->cmd);
    put_u16(f, 1, static_cast<std::uint16_t>(m->value));
  } else if (std::holds_alternative<stm::StopCmd>(command)) {
    f.id = id::kStop;
  } else {
    f.id = id::kImuGet;
  }
  return f;
}

std::optional<std::string> frame_to_command_line(const Frame& f) {
  if (f.id == id::kStop) return f.dlc == 0 ? "STOP" : "BAD_CAN_FRAME";
  if (f.id == id::kImuGet) return f.dlc == 0 ? "IMU,GET" : "BAD_CAN_FRAME";
  if (f.id != id::kMove) return std::nullopt;
  if (f.dlc != 3 || f.data[0] >= mdp::kNumCmds) return "BAD_CAN_FRAME";
  const auto cmd = static_cast<mdp::Cmd>(f.data[0]);
  const int value = get_u16(f, 1);
  if (value > mdp::max_value(cmd)) return "BAD_CAN_FRAME";
  return stm::encode_command(stm::Command{stm::MoveCmd{cmd, value}});
}

std::vector<Frame> line_to_frames(const std::string& line) {
  std::vector<Frame> out;
  const auto reply = stm::parse_reply(line);
  if (!reply) return out;

  Frame f;
  if (std::holds_alternative<stm::Ack>(*reply)) {
    f.id = id::kReply;
    f.dlc = 1;
    f.data[0] = kAck;
    out.push_back(f);
  } else if (std::holds_alternative<stm::AckStopped>(*reply)) {
    f.id = id::kReply;
    f.dlc = 1;
    f.data[0] = kAckStopped;
    out.push_back(f);
  } else if (const auto* e = std::get_if<stm::Err>(&*reply)) {
    f.id = id::kReply;
    f.dlc = 2;
    f.data[0] = kErr;
    f.data[1] = err_to_code(e->code);
    out.push_back(f);
  } else if (const auto* imu = std::get_if<stm::ImuReading>(&*reply)) {
    f.id = id::kImuReading;
    f.dlc = 2;
    put_u16(f, 0, static_cast<std::uint16_t>(clamp_i16(std::lround(imu->heading_deg * 100.0))));
    out.push_back(f);
  } else if (const auto* t = std::get_if<stm::Telemetry>(&*reply)) {
    Frame a;
    a.id = id::kTelemetryA;
    a.dlc = 8;
    put_u32(a, 0, static_cast<std::uint32_t>(static_cast<std::int32_t>(t->left_counts)));
    put_u32(a, 4, static_cast<std::uint32_t>(static_cast<std::int32_t>(t->right_counts)));
    Frame b;
    b.id = id::kTelemetryB;
    b.dlc = 6;
    put_u32(b, 0, static_cast<std::uint32_t>(t->t_ms));
    put_u16(b, 4, static_cast<std::uint16_t>(clamp_i16(std::lround(t->heading_deg * 100.0))));
    out.push_back(a);
    out.push_back(b);
  } else if (const auto* ev = std::get_if<stm::EventLine>(&*reply)) {
    const std::string& text = ev->text;
    if (text.rfind("MM,EV:START", 0) == 0) {
      const auto d = field_text(text, "D");
      const auto cmd = d ? cmd_from_name(*d) : std::nullopt;
      if (cmd) {
        f.id = id::kEventStart;
        f.dlc = 1;
        f.data[0] = *cmd;
        out.push_back(f);
      }
    } else if (text.rfind("MM,EV:END", 0) == 0) {
      const auto d = field_text(text, "D");
      const auto t = field_text(text, "T");
      const auto result = field_text(text, "RESULT");
      const auto dl = field_text(text, "DL");
      const auto dr = field_text(text, "DR");
      const auto cmd = d ? cmd_from_name(*d) : std::nullopt;
      if (cmd && t && result && dl && dr) {
        Frame a;
        a.id = id::kEventEndA;
        a.dlc = 6;
        a.data[0] = *cmd;
        a.data[1] = *result == "OK"        ? kOk
                    : *result == "TIMEOUT" ? kTimeout
                    : *result == "STOPPED" ? kStopped
                                           : kOtherResult;
        put_u32(a, 2, static_cast<std::uint32_t>(std::strtoul(t->c_str(), nullptr, 10)));
        Frame b;
        b.id = id::kEventEndB;
        b.dlc = 8;
        put_u32(b, 0,
                static_cast<std::uint32_t>(
                    static_cast<std::int32_t>(std::strtol(dl->c_str(), nullptr, 10))));
        put_u32(b, 4,
                static_cast<std::uint32_t>(
                    static_cast<std::int32_t>(std::strtol(dr->c_str(), nullptr, 10))));
        out.push_back(a);
        out.push_back(b);
      }
    }
  }
  return out;
}

std::vector<std::string> HostDecoder::feed(const Frame& f) {
  std::vector<std::string> lines;
  switch (f.id) {
    case id::kReply:
      if (f.dlc >= 1 && f.data[0] == kAck)
        lines.push_back("ACK");
      else if (f.dlc >= 1 && f.data[0] == kAckStopped)
        lines.push_back("ACK,STOPPED");
      else if (f.dlc >= 2 && f.data[0] == kErr)
        lines.push_back(code_to_err_line(f.data[1]));
      else
        ++dropped_;
      break;
    case id::kImuReading:
      if (f.dlc == 2) {
        const auto centi = static_cast<std::int16_t>(get_u16(f, 0));
        lines.push_back(stm::encode_reply(stm::ImuReading{centi / 100.0}));
      } else {
        ++dropped_;
      }
      break;
    case id::kEventStart:
      if (f.dlc == 1 && f.data[0] < mdp::kNumCmds) {
        lines.push_back(std::string("MM,EV:START,D:") +
                        mdp::cmd_name(static_cast<mdp::Cmd>(f.data[0])));
      } else {
        ++dropped_;
      }
      break;
    case id::kEventEndA:
      if (f.dlc == 6)
        end_ = PendingEnd{f.data[0], f.data[1], get_u32(f, 2)};
      else
        ++dropped_;
      break;
    case id::kEventEndB:
      if (f.dlc == 8 && end_ && end_->cmd < mdp::kNumCmds) {
        const char* result = end_->result == kOk        ? "OK"
                             : end_->result == kTimeout ? "TIMEOUT"
                             : end_->result == kStopped ? "STOPPED"
                                                        : "UNKNOWN";
        lines.push_back(std::string("MM,EV:END,D:") +
                        mdp::cmd_name(static_cast<mdp::Cmd>(end_->cmd)) +
                        ",T:" + std::to_string(end_->t_ms) + ",RESULT:" + result +
                        ",DL:" + std::to_string(static_cast<std::int32_t>(get_u32(f, 0))) +
                        ",DR:" + std::to_string(static_cast<std::int32_t>(get_u32(f, 4))));
      } else {
        ++dropped_;  // second half without its first half
      }
      end_.reset();
      break;
    case id::kTelemetryA:
      if (f.dlc == 8) {
        telemetry_ = PendingTelemetry{static_cast<std::int32_t>(get_u32(f, 0)),
                                      static_cast<std::int32_t>(get_u32(f, 4))};
      } else {
        ++dropped_;
      }
      break;
    case id::kTelemetryB:
      if (f.dlc == 6 && telemetry_) {
        const auto heading = static_cast<std::int16_t>(get_u16(f, 4));
        lines.push_back(
            stm::encode_reply(stm::Telemetry{static_cast<long>(get_u32(f, 0)), telemetry_->left,
                                             telemetry_->right, heading / 100.0}));
      } else {
        ++dropped_;
      }
      telemetry_.reset();
      break;
    default:
      ++dropped_;
  }
  return lines;
}

}  // namespace can
