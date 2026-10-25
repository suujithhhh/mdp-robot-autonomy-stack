// In-memory transport: the driver core on one end, the virtual STM32 on the other, driven by a
// simulated clock (so a 6-minute mission runs in milliseconds and tests are deterministic).
#pragma once

#include <algorithm>
#include <functional>

#include "mdp_mission/mission.hpp"
#include "virtual_stm32/virtual_stm.hpp"

namespace mission {

class SimLink : public Link {
 public:
  SimLink(const mdp::Pose& start, const stm::FaultConfig& faults = {},
          const stm::BridgeConfig& bridge_cfg = {})
      : stm_(start, faults), bridge_(start, bridge_cfg) {}

  stm::BridgeCore& bridge() override { return bridge_; }
  double now_s() const override { return t_; }
  stm::VirtualStm& device() { return stm_; }

  // Deliver the device's output in chunks of at most this many bytes (0 = all at once), which
  // exercises line reassembly the way a real serial read would.
  void set_read_chunk(std::size_t bytes) { chunk_ = bytes; }
  // Bytes written by the host pass through this filter before reaching the device (fault tests).
  void set_tx_filter(std::function<std::string(const std::string&)> f) {
    tx_filter_ = std::move(f);
  }
  // Called after every 10 ms step with (time, true pose, odometry pose); used to record paths.
  using Observer = std::function<void(double, const mdp::Pose&, const mdp::Pose&)>;
  void set_observer(Observer o) { observer_ = std::move(o); }

  void advance(double dt_s) override {
    double remaining = dt_s;
    while (remaining > 1e-12) {
      const double dt = std::min(remaining, 0.01);
      std::string tx = bridge_.take_tx();
      if (tx_filter_) tx = tx_filter_(tx);
      stm_.receive(tx);
      stm_.step(dt);
      t_ += dt;
      deliver(stm_.take_output());
      bridge_.poll(t_);
      if (observer_) observer_(t_, stm_.true_pose(), bridge_.odometry().pose);
      remaining -= dt;
    }
  }

 private:
  void deliver(const std::string& bytes) {
    if (chunk_ == 0) {
      bridge_.on_bytes(bytes, t_);
      return;
    }
    for (std::size_t i = 0; i < bytes.size(); i += chunk_)
      bridge_.on_bytes(bytes.substr(i, chunk_), t_);
  }

  stm::VirtualStm stm_;
  stm::BridgeCore bridge_;
  double t_{0.0};
  std::size_t chunk_{0};
  std::function<std::string(const std::string&)> tx_filter_;
  Observer observer_;
};

}  // namespace mission
