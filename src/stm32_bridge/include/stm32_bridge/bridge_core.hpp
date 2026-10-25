// Driver logic for the STM32 link, independent of ROS and of the operating system.
//
// BridgeCore owns the protocol state machine (one command in flight, terminal-reply matching,
// timeouts, STOP handling) and the dead-reckoning odometry. It never touches a serial port: the
// caller feeds it received bytes with on_bytes(), asks it for bytes to transmit with take_tx(),
// and supplies the current time. The ROS 2 node (ros/stm32_bridge_node.cpp) and the unit tests
// are two different callers of the same class, so everything below is testable without ROS.
//
// Rules taken from PROTOCOL.md sections 5-6:
//   * one command in flight; a second send_move() is refused locally (never sent -> no ERR,BUSY)
//   * only ACK / ACK,STOPPED / ERR,... are terminal; telemetry is filtered, never a reply
//   * no reply at all is ambiguous: report LinkTimeout, send a confirming STOP, mark the pose
//     uncertain, and NEVER resend the command automatically
#pragma once

#include <deque>
#include <optional>
#include <string>
#include <string_view>

#include "stm32_protocol/protocol.hpp"

namespace stm {

enum class Outcome {
  Ack,          // movement completed
  Stopped,      // ACK,STOPPED
  ErrBusy,      // device was not idle
  ErrBadCmd,
  ErrImuNotReady,
  ErrMotionTimeout,  // firmware deadline expired: pose unknown
  ErrOther,
  LinkTimeout,       // no terminal reply within the deadline
};
const char* outcome_name(Outcome o);

struct CommandResult {
  std::string wire;     // the command that was sent, e.g. "MOV,FL,90"
  Outcome outcome;
  double elapsed_s;
  bool pose_uncertain;  // true once the pose can no longer be trusted
};

struct BridgeConfig {
  // Wait this long beyond the firmware's own deadline before declaring a link timeout.
  double link_margin_s{1.5};
  // After a link timeout, wait this long for the confirming ACK,STOPPED.
  double stop_confirm_s{1.0};
  double counts_per_cm{75.0};
  // After each ACK, read the IMU at rest (IMU,GET) to correct the heading for the last telemetry
  // interval, as the Pi does before a turn (PROTOCOL.md section 4.1.1).
  bool refresh_imu_after_ack{true};
  mdp::PlannerConfig planner{};
};

struct Odometry {
  mdp::Pose pose{};          // integrated from encoder deltas + IMU heading (telemetry, ~10 Hz)
  double v_cm_s{0.0};        // speed estimated over the last telemetry interval
  long telemetry_count{0};
};

class BridgeCore {
 public:
  explicit BridgeCore(const mdp::Pose& initial, const BridgeConfig& cfg = {});

  // False (and nothing transmitted) if a command is already in flight or a STOP is outstanding.
  bool send_move(const MoveCmd& m, double now_s);
  void send_stop(double now_s);
  void send_imu_get(double now_s);

  void on_bytes(std::string_view bytes, double now_s);
  void poll(double now_s);  // call regularly: detects link timeouts

  std::string take_tx();
  std::optional<CommandResult> take_result();

  bool busy() const { return in_flight_.has_value() || stop_outstanding_; }
  bool pose_uncertain() const { return pose_uncertain_; }
  // Pose obtained by applying each ACKed command's ideal motion (the contract's definition of the
  // planner's estimate). Does not see slip; the odometry below does.
  const mdp::Pose& commanded_pose() const { return commanded_pose_; }
  const Odometry& odometry() const { return odom_; }
  std::optional<double> last_imu_deg() const { return last_imu_deg_; }

  // Counters for diagnostics.
  long unparsed_lines() const { return unparsed_lines_; }
  long unexpected_replies() const { return unexpected_replies_; }
  long overflow_lines() const { return framer_.overflow_count(); }

 private:
  struct InFlight {
    MoveCmd move;
    std::string wire;
    double sent_s;
    double link_deadline_s;
  };

  void write_line(const std::string& line);
  void resolve(Outcome o, double now_s);
  void on_reply(const Reply& r, double now_s);
  void on_telemetry(const Telemetry& t);
  void on_event(const std::string& text);
  void advance_odometry(long dl, long dr, double imu_deg);

  BridgeConfig cfg_;
  LineFramer framer_{kMaxHostLineChars};
  std::string tx_;
  std::deque<CommandResult> results_;

  std::optional<InFlight> in_flight_;
  bool stop_outstanding_{false};
  double stop_sent_s_{0.0};
  bool stop_is_confirmation_{false};  // STOP sent by us after a link timeout, not by the caller

  mdp::Pose commanded_pose_;
  Odometry odom_;
  bool pose_uncertain_{false};
  std::optional<double> last_imu_deg_;

  // Odometry state. Telemetry counts are displacement since the movement started, so the bridge
  // remembers how much of the current movement it has already applied.
  long move_applied_left_{0}, move_applied_right_{0};
  long last_tel_ms_{0};
  bool saw_start_{false};          // MM,EV:START seen for the command in flight
  double imu_ref_deg_{0.0};        // IMU reading taken at rest, when the heading was initial_
  bool imu_ref_set_{false};
  mdp::Pose initial_;

  long unparsed_lines_{0};
  long unexpected_replies_{0};
};

}  // namespace stm
