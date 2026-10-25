#include "can_bus/bus.hpp"

#include <algorithm>
#include <vector>

namespace can {

namespace {

constexpr int kErrorFrameBits = 6 + 8;  // error flag + error delimiter
constexpr int kIntermissionBits = 3;
constexpr int kBusOffThreshold = 255;
constexpr int kErrorPassiveThreshold = 127;

// Bit `pos` of the arbitration field: SOF first (always dominant), then the identifier MSB first.
int arbitration_bit(const Frame& f, int pos) {
  if (pos == 0) return 0;
  return (f.id >> (11 - pos)) & 1;
}

}  // namespace

const char* node_state_name(NodeState s) {
  switch (s) {
    case NodeState::ErrorActive:
      return "error-active";
    case NodeState::ErrorPassive:
      return "error-passive";
    case NodeState::BusOff:
      return "bus-off";
  }
  return "?";
}

bool Node::send(const Frame& frame) {
  if (tx_queue_.size() >= tx_capacity_) {
    ++tx_overflows_;
    return false;
  }
  tx_queue_.push_back(frame);
  return true;
}

std::optional<Frame> Node::receive() {
  if (rx_queue_.empty()) return std::nullopt;
  Frame f = rx_queue_.front();
  rx_queue_.pop_front();
  return f;
}

void Node::reset_bus_off() {
  bus_off_ = false;
  tec_ = 0;
  rec_ = 0;
}

NodeState Node::state() const {
  if (bus_off_) return NodeState::BusOff;
  if (tec_ > kErrorPassiveThreshold || rec_ > kErrorPassiveThreshold)
    return NodeState::ErrorPassive;
  return NodeState::ErrorActive;
}

Node& Bus::add_node(const std::string& name, std::size_t tx_capacity) {
  nodes_.emplace_back(name, tx_capacity);
  return nodes_.back();
}

double Bus::utilisation() const {
  if (now_s_ <= 0.0) return 0.0;
  return std::min(1.0, (stats_.busy_bits / cfg_.bitrate_bps) / now_s_);
}

void Bus::charge(long bits) {
  stats_.busy_bits += bits;
  now_s_ += bits / cfg_.bitrate_bps;
}

void Bus::advance_to(double t_s) {
  while (now_s_ < t_s) {
    if (!run_one_frame()) {
      now_s_ = t_s;  // nothing to send: the bus idles until t
      return;
    }
  }
}

bool Bus::run_one_frame() {
  std::vector<Node*> contenders;
  for (auto& n : nodes_) {
    if (n.enabled_ && !n.bus_off_ && !n.tx_queue_.empty()) contenders.push_back(&n);
  }
  if (contenders.empty()) return false;

  // Bitwise arbitration over SOF + the 11 identifier bits.
  std::vector<Node*> alive = contenders;
  for (int pos = 0; pos < 12 && alive.size() > 1; ++pos) {
    int bus_bit = 1;
    for (Node* n : alive) bus_bit &= arbitration_bit(n->tx_queue_.front(), pos);
    alive.erase(std::remove_if(
                    alive.begin(), alive.end(),
                    [&](Node* n) { return arbitration_bit(n->tx_queue_.front(), pos) != bus_bit; }),
                alive.end());
  }
  stats_.arbitration_losses += static_cast<long>(contenders.size() - alive.size());

  if (alive.size() > 1) {
    // Two nodes transmitting the same identifier is a design error on a real bus. Model what
    // happens electrically: their data/CRC bits collide, so both see a bit error.
    for (Node* n : alive) {
      n->tec_ += 8;
      ++n->tx_errors_;
      if (n->tec_ > kBusOffThreshold) n->bus_off_ = true;
    }
    ++stats_.frame_errors;
    charge(kErrorFrameBits + kIntermissionBits + 20);
    return true;
  }

  Node& tx = *alive.front();
  const Frame frame = tx.tx_queue_.front();
  Bits wire = encode(frame);

  // Random bit errors hit the stuffed region (the part receivers check).
  if (cfg_.bit_error_rate > 0.0) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    const std::size_t stuffed_len = wire.size() - kTrailerBits;
    for (std::size_t i = 0; i < stuffed_len; ++i) {
      if (u(rng_) < cfg_.bit_error_rate) wire[i] ^= 1;
    }
  }

  std::vector<Node*> receivers;
  for (auto& n : nodes_) {
    if (&n != &tx && n.enabled_ && !n.bus_off_) receivers.push_back(&n);
  }

  const DecodeResult decoded = decode(wire);
  const long frame_bits = static_cast<long>(wire.size());
  const long stuffed_bits = frame_bits - kTrailerBits;

  if (decoded.error != DecodeError::None) {
    // Receivers flag the error; the frame is destroyed on the bus and will be retransmitted.
    ++stats_.frame_errors;
    tx.tec_ += 8;
    ++tx.tx_errors_;
    for (Node* r : receivers) r->rec_ += 1;
    if (tx.tec_ > kBusOffThreshold) tx.bus_off_ = true;
    charge(stuffed_bits + kErrorFrameBits + kIntermissionBits);
    return true;
  }

  if (receivers.empty()) {
    // The frame was fine but no node drove the ACK slot dominant: ACK error.
    ++stats_.ack_errors;
    tx.tec_ += 8;
    ++tx.tx_errors_;
    if (tx.tec_ > kBusOffThreshold) tx.bus_off_ = true;
    charge(stuffed_bits + 2 + kErrorFrameBits + kIntermissionBits);
    return true;
  }

  // Success: every receiver gets the decoded frame, the transmitter's queue advances.
  ++stats_.frames_ok;
  for (Node* r : receivers) {
    r->rx_queue_.push_back(decoded.frame);
    ++r->frames_received_;
    if (r->rec_ > 0) --r->rec_;
  }
  tx.tx_queue_.pop_front();
  ++tx.frames_sent_;
  if (tx.tec_ > 0) --tx.tec_;
  charge(frame_bits);
  return true;
}

}  // namespace can
