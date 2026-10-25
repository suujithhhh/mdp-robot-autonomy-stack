// A software model of the MDP robot's STM32 motion controller.
//
// It speaks the real UART protocol (PROTOCOL.md section 4): one command in flight, exactly one
// terminal reply per command (ACK / ERR,<code>), a priority STOP, a motion deadline of
// 5 s + 0.15 s/cm (straights) or 5 s + 0.10 s/deg (turns), and ~10 Hz telemetry while moving.
//
// The model is transport-agnostic: feed it bytes with receive(), advance simulated time with
// step(), and read what it wants to transmit with take_output(). A pseudo-terminal front end
// (app/virtual_stm32_main.cpp) wires it to a real serial-style device on Linux; the unit tests
// connect it straight to the driver core in memory, with a simulated clock.
//
// Simulator assumptions (NOT measured on the robot, documented so nothing is over-claimed):
//   * motion durations use the timings measured on the robot (config.py of the Python planner):
//     straight 1.43 s + 0.0215 s/cm, turn 0.73 s + 0.020 s/deg; progress is linear in time
//   * encoders: 75 counts/cm; wheel track 15 cm for the inner/outer wheel split in turns;
//     telemetry and the MM,EV:END line report displacement since the movement started
//     (the END line carries DL/DR as in PROTOCOL.md section 4.7, a TARGET feature of the real
//     firmware)
//   * IMU: true heading + constant bias + Gaussian noise, reported in degrees
#pragma once

#include <random>
#include <string>
#include <string_view>

#include "mdp_planner_core/primitives.hpp"
#include "stm32_protocol/protocol.hpp"

namespace stm {

struct FaultConfig {
  double drop_rate{0.0};      // probability that each outgoing byte is lost
  bool imu_fail{false};       // MOV / IMU,GET answer ERR,IMU_NOT_READY
  bool stall{false};          // motors never finish: the motion deadline expires
  double encoder_noise{0.0};  // relative Gaussian noise on encoder counts (e.g. 0.01 = 1 %)
  double imu_noise_deg{0.0};  // Gaussian noise on each reported IMU heading, degrees
  double imu_bias_deg{0.0};   // constant bias added to the reported heading, degrees
  double slip{0.0};           // fraction of commanded travel that is lost (wheel slip), 0..1
  unsigned seed{1};
};

class VirtualStm {
 public:
  explicit VirtualStm(const mdp::Pose& initial, const FaultConfig& faults = {},
                      const mdp::PlannerConfig& cfg = {});

  // Bytes arriving from the host (may be partial lines).
  void receive(std::string_view bytes);
  // Advance simulated time by dt seconds (internally sub-stepped to 10 ms).
  void step(double dt_s);
  // Bytes the device has transmitted since the last call (after fault-injected byte loss).
  std::string take_output();

  const mdp::Pose& true_pose() const { return pose_; }
  double now_s() const { return now_s_; }
  bool moving() const { return state_ != State::Idle; }
  // Total encoder counts since power-up (what the real firmware's counters would hold).
  long left_counts() const { return left_counts_; }
  long right_counts() const { return right_counts_; }

 private:
  enum class State { Idle, Moving, Braking };

  void handle_line(const std::string& line);
  void start_move(const MoveCmd& m);
  void tick(double dt);
  void finish_move(const char* result_text, const Reply& terminal);
  void emit(const Reply& r);
  void emit_raw(const std::string& line);
  void update_pose_and_encoders(double fraction);
  double reported_heading_deg();

  mdp::PlannerConfig cfg_;
  FaultConfig faults_;
  std::mt19937 rng_;
  LineFramer framer_;
  std::string out_;

  mdp::Pose pose_;        // true pose (rear-axle midpoint)
  mdp::Pose move_start_;  // pose when the current move began
  State state_{State::Idle};
  MoveCmd move_{mdp::Cmd::F, 0};
  double now_s_{0.0};
  double move_started_s_{0.0};
  double move_duration_s_{0.0};
  double deadline_s_{0.0};
  double next_telemetry_s_{0.0};
  double brake_end_s_{0.0};
  double last_fraction_{0.0};
  bool stop_pending_{false};
  double stop_reply_at_s_{0.0};
  bool stop_idle_reply_{false};

  long left_counts_{0}, right_counts_{0};
  long move_left_start_{0}, move_right_start_{0};
};

}  // namespace stm
