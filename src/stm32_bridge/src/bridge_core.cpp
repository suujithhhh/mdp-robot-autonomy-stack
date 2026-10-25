#include "stm32_bridge/bridge_core.hpp"

#include <cmath>
#include <cstdlib>

namespace stm {

namespace {

// The firmware's own deadline (PROTOCOL.md section 5): 5 s + 0.15 s/cm or 5 s + 0.10 s/deg.
double firmware_deadline_s(const MoveCmd& m) {
  return mdp::is_straight(m.cmd) ? 5.0 + 0.15 * m.value : 5.0 + 0.10 * m.value;
}

// Pulls "DL:<n>" / "DR:<n>" out of an MM,EV:END line.
std::optional<long> field(std::string_view line, std::string_view key) {
  const std::string needle = std::string(",") + std::string(key) + ":";
  const std::size_t at = line.find(needle);
  if (at == std::string_view::npos) return std::nullopt;
  const std::string rest(line.substr(at + needle.size()));
  char* end = nullptr;
  const long v = std::strtol(rest.c_str(), &end, 10);
  if (end == rest.c_str()) return std::nullopt;
  return v;
}

}  // namespace

const char* outcome_name(Outcome o) {
  switch (o) {
    case Outcome::Ack: return "ACK";
    case Outcome::Stopped: return "STOPPED";
    case Outcome::ErrBusy: return "ERR_BUSY";
    case Outcome::ErrBadCmd: return "ERR_BAD_CMD";
    case Outcome::ErrImuNotReady: return "ERR_IMU_NOT_READY";
    case Outcome::ErrMotionTimeout: return "ERR_MOTION_TIMEOUT";
    case Outcome::ErrOther: return "ERR_OTHER";
    case Outcome::LinkTimeout: return "LINK_TIMEOUT";
  }
  return "?";
}

BridgeCore::BridgeCore(const mdp::Pose& initial, const BridgeConfig& cfg)
    : cfg_(cfg), commanded_pose_(initial), initial_(initial) {
  odom_.pose = initial;
}

void BridgeCore::write_line(const std::string& line) { tx_ += line + "\n"; }

std::string BridgeCore::take_tx() {
  std::string out;
  out.swap(tx_);
  return out;
}

std::optional<CommandResult> BridgeCore::take_result() {
  if (results_.empty()) return std::nullopt;
  CommandResult r = results_.front();
  results_.pop_front();
  return r;
}

bool BridgeCore::send_move(const MoveCmd& m, double now_s) {
  if (busy()) return false;  // one command in flight: refuse locally instead of provoking ERR,BUSY
  InFlight f;
  f.move = m;
  f.wire = encode_command(Command{m});
  f.sent_s = now_s;
  f.link_deadline_s = now_s + firmware_deadline_s(m) + cfg_.link_margin_s;
  in_flight_ = f;
  saw_start_ = false;
  move_applied_left_ = move_applied_right_ = 0;
  write_line(f.wire);
  return true;
}

void BridgeCore::send_stop(double now_s) {
  if (stop_outstanding_) return;  // coalesced, as on the device
  stop_outstanding_ = true;
  stop_is_confirmation_ = false;
  stop_sent_s_ = now_s;
  write_line("STOP");
}

void BridgeCore::send_imu_get(double) { write_line("IMU,GET"); }

void BridgeCore::resolve(Outcome o, double now_s) {
  if (!in_flight_) return;
  results_.push_back({in_flight_->wire, o, now_s - in_flight_->sent_s, pose_uncertain_});
  in_flight_.reset();
}

void BridgeCore::on_bytes(std::string_view bytes, double now_s) {
  for (const auto& line : framer_.feed(bytes)) {
    const auto reply = parse_reply(line);
    if (!reply) {
      ++unparsed_lines_;
      continue;
    }
    on_reply(*reply, now_s);
  }
}

void BridgeCore::on_reply(const Reply& r, double now_s) {
  if (std::holds_alternative<Ack>(r)) {
    if (!in_flight_) {
      ++unexpected_replies_;
      return;
    }
    // ACK confirms completion, not measured position: advance the estimate by the ideal motion.
    commanded_pose_ =
        mdp::propagate(commanded_pose_, in_flight_->move.cmd, in_flight_->move.value, cfg_.planner);
    resolve(Outcome::Ack, now_s);
    if (cfg_.refresh_imu_after_ack && imu_ref_set_) send_imu_get(now_s);
  } else if (std::holds_alternative<AckStopped>(r)) {
    if (!stop_outstanding_) {
      ++unexpected_replies_;
      return;
    }
    stop_outstanding_ = false;
    if (stop_is_confirmation_) {
      stop_is_confirmation_ = false;  // the link timeout was already reported
      return;
    }
    if (in_flight_) {
      pose_uncertain_ = true;  // a STOP that crossed a movement: the pose is no longer known
      resolve(Outcome::Stopped, now_s);
    } else {
      results_.push_back({"STOP", Outcome::Stopped, now_s - stop_sent_s_, pose_uncertain_});
    }
  } else if (const auto* e = std::get_if<Err>(&r)) {
    if (!in_flight_) {
      ++unexpected_replies_;
      return;
    }
    Outcome o = Outcome::ErrOther;
    switch (e->code) {
      case ErrCode::BadCmd: o = Outcome::ErrBadCmd; break;
      case ErrCode::Busy: o = Outcome::ErrBusy; break;
      case ErrCode::ImuNotReady:
        o = Outcome::ErrImuNotReady;
        if (saw_start_) pose_uncertain_ = true;  // aborted mid-motion; if rejected up front, nothing moved
        break;
      case ErrCode::MotionTimeout:
        o = Outcome::ErrMotionTimeout;
        pose_uncertain_ = true;
        break;
      case ErrCode::Unknown: break;
    }
    resolve(o, now_s);
  } else if (const auto* imu = std::get_if<ImuReading>(&r)) {
    last_imu_deg_ = imu->heading_deg;
    if (!imu_ref_set_ && !in_flight_) {  // reference taken while the robot is standing still
      imu_ref_deg_ = imu->heading_deg;
      imu_ref_set_ = true;
    } else if (imu_ref_set_ && (!in_flight_ || !saw_start_)) {
      // A reading taken before the current movement started (the device answers IMU,GET before
      // it starts the next MOV): the robot was at rest, so it fixes the heading exactly.
      odom_.pose.heading = mdp::normalize_heading(
          (initial_.heading * mdp::kRadToDeg + (imu->heading_deg - imu_ref_deg_)) * mdp::kDegToRad);
    }
  } else if (const auto* t = std::get_if<Telemetry>(&r)) {
    on_telemetry(*t);
  } else if (const auto* ev = std::get_if<EventLine>(&r)) {
    on_event(ev->text);
  }
}

// Advance the odometry by wheel-count deltas, with the heading taken from the IMU.
void BridgeCore::advance_odometry(long dl, long dr, double imu_deg) {
  const double ref = imu_ref_set_ ? imu_ref_deg_ : 0.0;
  const double new_heading = mdp::normalize_heading((initial_.heading * mdp::kRadToDeg + (imu_deg - ref)) * mdp::kDegToRad);
  // Midpoint heading over the interval: exact enough at 10 Hz and robust across the +/-pi wrap.
  const double mid = odom_.pose.heading + mdp::normalize_heading(new_heading - odom_.pose.heading) / 2.0;
  const double dist_cm = (static_cast<double>(dl) + static_cast<double>(dr)) / 2.0 / cfg_.counts_per_cm;
  odom_.pose.x += dist_cm * std::cos(mid);
  odom_.pose.y += dist_cm * std::sin(mid);
  odom_.pose.heading = new_heading;
}

void BridgeCore::on_telemetry(const Telemetry& t) {
  // Counts are displacement since the movement started; use the increment since the last report.
  const long dl = t.left_counts - move_applied_left_;
  const long dr = t.right_counts - move_applied_right_;
  move_applied_left_ = t.left_counts;
  move_applied_right_ = t.right_counts;
  const double dt_s = (t.t_ms - last_tel_ms_) / 1000.0;
  last_tel_ms_ = t.t_ms;
  advance_odometry(dl, dr, t.heading_deg);
  if (dt_s > 0.0) odom_.v_cm_s = (static_cast<double>(dl) + dr) / 2.0 / cfg_.counts_per_cm / dt_s;
  ++odom_.telemetry_count;
  last_imu_deg_ = t.heading_deg;
}

void BridgeCore::on_event(const std::string& text) {
  if (text.rfind("MM,EV:START", 0) == 0) {
    saw_start_ = true;
    last_tel_ms_ = 0;
    return;
  }
  if (text.rfind("MM,EV:END", 0) == 0) {
    // The END line carries the final displacement; apply what the last telemetry did not cover.
    const auto dl = field(text, "DL");
    const auto dr = field(text, "DR");
    if (dl && dr && last_imu_deg_) {
      advance_odometry(*dl - move_applied_left_, *dr - move_applied_right_, *last_imu_deg_);
      move_applied_left_ = *dl;
      move_applied_right_ = *dr;
    }
    odom_.v_cm_s = 0.0;
  }
}

void BridgeCore::poll(double now_s) {
  if (in_flight_ && now_s >= in_flight_->link_deadline_s) {
    // No terminal reply at all: the command may or may not have executed. Report it, mark the pose
    // uncertain, send a confirming STOP, and never resend the command.
    pose_uncertain_ = true;
    resolve(Outcome::LinkTimeout, now_s);
    if (!stop_outstanding_) {
      stop_outstanding_ = true;
      stop_is_confirmation_ = true;
      stop_sent_s_ = now_s;
      write_line("STOP");
    }
  }
  if (stop_outstanding_ && now_s - stop_sent_s_ > cfg_.stop_confirm_s && stop_is_confirmation_) {
    stop_outstanding_ = false;  // gave up waiting for the confirmation
    stop_is_confirmation_ = false;
  }
}

}  // namespace stm
