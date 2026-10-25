#include "stm32_protocol/protocol.hpp"

#include <charconv>
#include <cstdio>

namespace stm {

namespace {

// Parses a non-negative decimal integer with no sign, no spaces and no leading junk.
std::optional<int> parse_uint(std::string_view s) {
  if (s.empty() || s.size() > 6) return std::nullopt;
  int v = 0;
  for (char ch : s) {
    if (ch < '0' || ch > '9') return std::nullopt;
    v = v * 10 + (ch - '0');
  }
  return v;
}

std::optional<mdp::Cmd> parse_cmd_name(std::string_view s) {
  for (int i = 0; i < mdp::kNumCmds; ++i) {
    if (s == mdp::cmd_name(static_cast<mdp::Cmd>(i))) return static_cast<mdp::Cmd>(i);
  }
  return std::nullopt;
}

std::vector<std::string_view> split(std::string_view s, char sep) {
  std::vector<std::string_view> parts;
  std::size_t start = 0;
  for (;;) {
    const std::size_t at = s.find(sep, start);
    if (at == std::string_view::npos) {
      parts.push_back(s.substr(start));
      return parts;
    }
    parts.push_back(s.substr(start, at - start));
    start = at + 1;
  }
}

// "T:123" -> 123 when the key matches.
std::optional<long> keyed_long(std::string_view part, std::string_view key) {
  if (part.size() <= key.size() + 1 || part.substr(0, key.size()) != key || part[key.size()] != ':')
    return std::nullopt;
  std::string_view num = part.substr(key.size() + 1);
  bool negative = false;
  if (!num.empty() && num[0] == '-') {
    negative = true;
    num.remove_prefix(1);
  }
  const auto v = parse_uint(num);
  if (!v) return std::nullopt;
  return negative ? -static_cast<long>(*v) : static_cast<long>(*v);
}

}  // namespace

std::string encode_command(const Command& c) {
  if (const auto* m = std::get_if<MoveCmd>(&c)) {
    return std::string("MOV,") + mdp::cmd_name(m->cmd) + "," + std::to_string(m->value);
  }
  if (std::holds_alternative<StopCmd>(c)) return "STOP";
  return "IMU,GET";
}

std::optional<Command> parse_command(std::string_view line) {
  if (line.size() > kMaxLineChars) return std::nullopt;
  if (line == "STOP") return Command{StopCmd{}};
  if (line == "IMU,GET") return Command{ImuGetCmd{}};
  const auto parts = split(line, ',');
  if (parts.size() != 3 || parts[0] != "MOV") return std::nullopt;
  const auto cmd = parse_cmd_name(parts[1]);
  const auto value = parse_uint(parts[2]);
  if (!cmd || !value) return std::nullopt;
  // The device accepts 0..max (the planner itself never emits 0); see PROTOCOL.md section 4.
  if (*value > mdp::max_value(*cmd)) return std::nullopt;
  return Command{MoveCmd{*cmd, *value}};
}

const char* err_code_name(ErrCode e) {
  switch (e) {
    case ErrCode::BadCmd:
      return "BAD_CMD";
    case ErrCode::Busy:
      return "BUSY";
    case ErrCode::ImuNotReady:
      return "IMU_NOT_READY";
    case ErrCode::MotionTimeout:
      return "MOTION_TIMEOUT";
    case ErrCode::Unknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

std::string encode_reply(const Reply& r) {
  if (std::holds_alternative<Ack>(r)) return "ACK";
  if (std::holds_alternative<AckStopped>(r)) return "ACK,STOPPED";
  if (const auto* e = std::get_if<Err>(&r)) {
    return std::string("ERR,") + (e->code == ErrCode::Unknown ? e->raw : err_code_name(e->code));
  }
  if (const auto* i = std::get_if<ImuReading>(&r)) {
    char buf[48];
    std::snprintf(buf, sizeof buf, "IMU,%.2f", i->heading_deg);
    return buf;
  }
  if (const auto* t = std::get_if<Telemetry>(&r)) {
    char buf[96];
    std::snprintf(buf, sizeof buf, "MM,TEL,T:%ld,L:%ld,R:%ld,H:%ld", t->t_ms, t->left_counts,
                  t->right_counts, std::lround(t->heading_deg * 100.0));
    return buf;
  }
  return std::get<EventLine>(r).text;
}

std::optional<Reply> parse_reply(std::string_view line) {
  if (line == "ACK") return Reply{Ack{}};
  if (line == "ACK,STOPPED") return Reply{AckStopped{}};
  if (line.substr(0, 4) == "ERR,") {
    const std::string_view code = line.substr(4);
    Err e{ErrCode::Unknown, std::string(code)};
    if (code == "BAD_CMD")
      e.code = ErrCode::BadCmd;
    else if (code == "BUSY")
      e.code = ErrCode::Busy;
    else if (code == "IMU_NOT_READY")
      e.code = ErrCode::ImuNotReady;
    else if (code == "MOTION_TIMEOUT")
      e.code = ErrCode::MotionTimeout;
    return Reply{e};
  }
  if (line.substr(0, 4) == "IMU,") {
    const std::string text(line.substr(4));
    char* end = nullptr;
    const double deg = std::strtod(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0') return std::nullopt;
    return Reply{ImuReading{deg}};
  }
  if (line.substr(0, 7) == "MM,TEL,") {
    const auto parts = split(line.substr(7), ',');
    if (parts.size() != 4) return std::nullopt;
    const auto t = keyed_long(parts[0], "T");
    const auto l = keyed_long(parts[1], "L");
    const auto r = keyed_long(parts[2], "R");
    const auto h = keyed_long(parts[3], "H");
    if (!t || !l || !r || !h) return std::nullopt;
    return Reply{Telemetry{*t, *l, *r, *h / 100.0}};
  }
  if (line.substr(0, 3) == "MM,") return Reply{EventLine{std::string(line)}};
  return std::nullopt;
}

bool is_terminal(const Reply& r) {
  return std::holds_alternative<Ack>(r) || std::holds_alternative<AckStopped>(r) ||
         std::holds_alternative<Err>(r);
}

std::vector<std::string> LineFramer::feed(std::string_view bytes) {
  std::vector<std::string> lines;
  for (char ch : bytes) {
    if (ch == '\n') {
      if (!discarding_) {
        if (!buffer_.empty() && buffer_.back() == '\r') buffer_.pop_back();
        lines.push_back(buffer_);
      }
      buffer_.clear();
      discarding_ = false;
    } else if (!discarding_) {
      buffer_.push_back(ch);
      if (buffer_.size() > max_chars_ + 1) {  // +1 leaves room for a trailing '\r'
        discarding_ = true;
        buffer_.clear();
        ++overflows_;
      }
    }
  }
  return lines;
}

}  // namespace stm
