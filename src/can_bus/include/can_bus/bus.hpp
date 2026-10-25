// A discrete-event model of one CAN bus segment and the controllers attached to it.
//
// What is modelled (per transmitted frame):
//   * bitwise arbitration: the bus is a wired-AND, a dominant 0 overrides a recessive 1, and a
//     node that sends 1 while the bus reads 0 loses and retries after the winner's frame
//   * the real bit stream (stuffing, CRC-15) and its duration at the configured bit rate
//   * random bit errors, detected by receivers through stuff / CRC / form checks
//   * the ACK slot: a frame nobody acknowledges is an ACK error and is retransmitted
//   * the transmit/receive error counters and the error-active / error-passive / bus-off states
//     (simplified: error counters follow the +8 / -1 rules; recovery from bus-off is explicit)
//
// Not modelled: analogue/physical layer, propagation delay, bit-level resynchronisation, the
// 8-bit suspend time of error-passive nodes, automatic bus-off recovery, hardware filters,
// multiple transmit mailboxes (each node sends its queue strictly first-in first-out).
#pragma once

#include <deque>
#include <optional>
#include <random>
#include <string>

#include "can_bus/frame.hpp"

namespace can {

enum class NodeState { ErrorActive, ErrorPassive, BusOff };
const char* node_state_name(NodeState s);

class Node {
 public:
  explicit Node(std::string name, std::size_t tx_capacity = 32)
      : name_(std::move(name)), tx_capacity_(tx_capacity) {}

  // Queue a frame for transmission. False (and the frame is dropped) if the queue is full.
  bool send(const Frame& frame);
  // Oldest received frame, if any.
  std::optional<Frame> receive();

  // A disabled node is unplugged: it neither transmits nor acknowledges.
  void set_enabled(bool enabled) { enabled_ = enabled; }
  bool enabled() const { return enabled_; }
  // Manual recovery from bus-off (real controllers wait for 128 x 11 recessive bits).
  void reset_bus_off();

  const std::string& name() const { return name_; }
  int tec() const { return tec_; }
  int rec() const { return rec_; }
  NodeState state() const;
  std::size_t tx_pending() const { return tx_queue_.size(); }
  long frames_sent() const { return frames_sent_; }
  long frames_received() const { return frames_received_; }
  long tx_overflows() const { return tx_overflows_; }
  long tx_errors() const { return tx_errors_; }

 private:
  friend class Bus;
  std::string name_;
  std::size_t tx_capacity_;
  std::deque<Frame> tx_queue_;
  std::deque<Frame> rx_queue_;
  bool enabled_{true};
  bool bus_off_{false};
  int tec_{0};
  int rec_{0};
  long frames_sent_{0};
  long frames_received_{0};
  long tx_overflows_{0};
  long tx_errors_{0};
};

class Bus {
 public:
  struct Config {
    double bitrate_bps{500000.0};
    double bit_error_rate{0.0};  // probability that any one transmitted bit is corrupted
    unsigned seed{1};
  };
  struct Stats {
    long frames_ok{0};
    long frame_errors{0};  // corrupted frames caught by receivers
    long ack_errors{0};    // frames nobody acknowledged
    long arbitration_losses{0};
    long busy_bits{0};
  };

  Bus() : Bus(Config{}) {}
  explicit Bus(const Config& cfg) : cfg_(cfg), rng_(cfg.seed) {}

  // Returned references stay valid for the life of the bus.
  Node& add_node(const std::string& name, std::size_t tx_capacity = 32);

  // Run the bus until simulated time t (frames in flight may finish slightly after t).
  void advance_to(double t_s);
  double now_s() const { return now_s_; }

  const Stats& stats() const { return stats_; }
  // Fraction of elapsed bus time spent carrying bits.
  double utilisation() const;

 private:
  bool run_one_frame();
  void charge(long bits);

  Config cfg_;
  std::mt19937 rng_;
  std::deque<Node> nodes_;
  Stats stats_;
  double now_s_{0.0};
};

}  // namespace can
