#include "virtual_stm32/virtual_stm.hpp"

#include <algorithm>
#include <cmath>

namespace stm {

namespace {

constexpr double kCountsPerCm = 75.0;
constexpr double kTrackCm = 15.0;           // simulator assumption: distance between rear wheels
constexpr double kTelemetryPeriodS = 0.1;   // 10 Hz
constexpr double kSubStepS = 0.01;          // the real motion task runs about every 10 ms
constexpr double kBrakeWindowS = 0.15;      // STOP: brake window while moving
constexpr double kIdleStopReplyS = 0.025;   // STOP at idle: reply after about 25 ms

// Durations measured on the robot (algo/config.py of the Python planner).
double nominal_duration_s(const MoveCmd& m) {
  if (m.value == 0) return 0.0;
  return mdp::is_straight(m.cmd) ? 1.43 + 0.0215 * m.value : 0.73 + 0.020 * m.value;
}

// The firmware's own deadline (PROTOCOL.md section 5).
double deadline_s(const MoveCmd& m) {
  return mdp::is_straight(m.cmd) ? 5.0 + 0.15 * m.value : 5.0 + 0.10 * m.value;
}

}  // namespace

VirtualStm::VirtualStm(const mdp::Pose& initial, const FaultConfig& faults,
                       const mdp::PlannerConfig& cfg)
    : cfg_(cfg), faults_(faults), rng_(faults.seed), pose_(initial), move_start_(initial) {}

void VirtualStm::receive(std::string_view bytes) {
  for (const auto& line : framer_.feed(bytes)) handle_line(line);
}

std::string VirtualStm::take_output() {
  std::string out;
  out.swap(out_);
  return out;
}

void VirtualStm::emit_raw(const std::string& line) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (char ch : line + "\n") {
    if (faults_.drop_rate > 0.0 && u(rng_) < faults_.drop_rate) continue;  // byte lost on the wire
    out_.push_back(ch);
  }
}

void VirtualStm::emit(const Reply& r) { emit_raw(encode_reply(r)); }

double VirtualStm::reported_heading_deg() {
  std::normal_distribution<double> noise(0.0, std::max(faults_.imu_noise_deg, 1e-12));
  const double n = faults_.imu_noise_deg > 0.0 ? noise(rng_) : 0.0;
  return pose_.heading * mdp::kRadToDeg + faults_.imu_bias_deg + n;
}

void VirtualStm::handle_line(const std::string& line) {
  const auto cmd = parse_command(line);
  if (!cmd) {
    emit(Err{ErrCode::BadCmd, ""});
    return;
  }
  if (std::holds_alternative<StopCmd>(*cmd)) {
    // STOP is never BUSY. A STOP while one is outstanding is coalesced (no second reply).
    if (stop_pending_) return;
    stop_pending_ = true;
    if (state_ == State::Idle) {
      stop_idle_reply_ = true;
      stop_reply_at_s_ = now_s_ + kIdleStopReplyS;
    } else if (state_ == State::Moving) {
      state_ = State::Braking;
      brake_end_s_ = now_s_ + kBrakeWindowS;
    }
    return;
  }
  if (std::holds_alternative<ImuGetCmd>(*cmd)) {
    if (faults_.imu_fail) emit(Err{ErrCode::ImuNotReady, ""});
    else emit(ImuReading{reported_heading_deg()});
    return;
  }
  // Motion commands: refused while moving, braking or while a STOP is outstanding.
  if (state_ != State::Idle || stop_pending_) {
    emit(Err{ErrCode::Busy, ""});
    return;
  }
  start_move(std::get<MoveCmd>(*cmd));
}

void VirtualStm::start_move(const MoveCmd& m) {
  if (faults_.imu_fail) {
    emit(Err{ErrCode::ImuNotReady, ""});  // rejected before any motion: nothing moved
    return;
  }
  if (m.value == 0) {
    emit(Ack{});
    return;
  }
  move_ = m;
  move_start_ = pose_;
  move_left_start_ = left_counts_;
  move_right_start_ = right_counts_;
  move_started_s_ = now_s_;
  move_duration_s_ = nominal_duration_s(m);
  deadline_s_ = now_s_ + deadline_s(m);
  next_telemetry_s_ = now_s_ + kTelemetryPeriodS;
  last_fraction_ = 0.0;
  state_ = State::Moving;
  emit_raw(std::string("MM,EV:START,D:") + mdp::cmd_name(m.cmd));
}

void VirtualStm::update_pose_and_encoders(double fraction) {
  fraction = std::clamp(fraction, 0.0, 1.0);
  const double effective = fraction * (1.0 - faults_.slip);
  pose_ = mdp::pose_at_fraction(move_start_, move_.cmd, move_.value, effective, cfg_);

  // Encoder counts for the travelled part of the primitive.
  const double sign = mdp::is_reverse(move_.cmd) ? -1.0 : 1.0;
  double left_cm, right_cm;
  if (mdp::is_straight(move_.cmd)) {
    left_cm = right_cm = sign * move_.value * effective;
  } else {
    const double radius = mdp::turn_radius_cm(move_.cmd, cfg_);
    const double theta = move_.value * mdp::kDegToRad * effective;
    // Left-hand turns (centre on the left): the left wheel is the inner wheel.
    const bool centre_on_left = (move_.cmd == mdp::Cmd::FL || move_.cmd == mdp::Cmd::BR);
    const double inner = (radius - kTrackCm / 2) * theta, outer = (radius + kTrackCm / 2) * theta;
    left_cm = sign * (centre_on_left ? inner : outer);
    right_cm = sign * (centre_on_left ? outer : inner);
  }
  std::normal_distribution<double> noise(1.0, std::max(faults_.encoder_noise, 1e-12));
  const double scale = faults_.encoder_noise > 0.0 ? noise(rng_) : 1.0;
  left_counts_ = move_left_start_ + std::lround(left_cm * kCountsPerCm * scale);
  right_counts_ = move_right_start_ + std::lround(right_cm * kCountsPerCm * scale);
  last_fraction_ = fraction;
}

void VirtualStm::finish_move(const char* result_text, const Reply& terminal) {
  const long t_ms = std::lround((now_s_ - move_started_s_) * 1000.0);
  emit_raw(std::string("MM,EV:END,D:") + mdp::cmd_name(move_.cmd) + ",T:" + std::to_string(t_ms) +
           ",RESULT:" + result_text + ",DL:" + std::to_string(left_counts_ - move_left_start_) +
           ",DR:" + std::to_string(right_counts_ - move_right_start_));
  emit(terminal);
  state_ = State::Idle;
}

void VirtualStm::tick(double dt) {
  now_s_ += dt;

  if (state_ == State::Moving) {
    if (!faults_.stall) {
      const double fraction = move_duration_s_ > 0 ? (now_s_ - move_started_s_) / move_duration_s_ : 1.0;
      update_pose_and_encoders(fraction);
    }
    if (now_s_ >= next_telemetry_s_) {
      next_telemetry_s_ += kTelemetryPeriodS;
      emit(Telemetry{std::lround((now_s_ - move_started_s_) * 1000.0), left_counts_ - move_left_start_,
                     right_counts_ - move_right_start_, reported_heading_deg()});
    }
    if (!faults_.stall && last_fraction_ >= 1.0) {
      finish_move("OK", Ack{});
    } else if (now_s_ >= deadline_s_) {
      finish_move("TIMEOUT", Err{ErrCode::MotionTimeout, ""});
    }
  } else if (state_ == State::Braking) {
    // The running movement is reported ONLY through ACK,STOPPED (its own ACK/ERR is never sent).
    if (now_s_ >= brake_end_s_) {
      const long t_ms = std::lround((now_s_ - move_started_s_) * 1000.0);
      emit_raw(std::string("MM,EV:END,D:") + mdp::cmd_name(move_.cmd) + ",T:" + std::to_string(t_ms) +
               ",RESULT:STOPPED,DL:" + std::to_string(left_counts_ - move_left_start_) +
               ",DR:" + std::to_string(right_counts_ - move_right_start_));
      emit(AckStopped{});
      state_ = State::Idle;
      stop_pending_ = false;
    }
  }

  if (stop_idle_reply_ && now_s_ >= stop_reply_at_s_) {
    emit(AckStopped{});
    stop_idle_reply_ = false;
    stop_pending_ = false;
  }
}

void VirtualStm::step(double dt_s) {
  double remaining = dt_s;
  while (remaining > 1e-12) {
    const double dt = std::min(remaining, kSubStepS);
    tick(dt);
    remaining -= dt;
  }
}

}  // namespace stm
