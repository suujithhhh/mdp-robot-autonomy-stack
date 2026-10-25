// The same driver and virtual STM32 as SimLink, but connected through a simulated CAN bus
// instead of a UART: commands become CAN frames, the device's replies, events and telemetry come
// back as CAN frames, and the host rebuilds the usual text lines from them. The driver code
// (BridgeCore) and the device model (VirtualStm) are unchanged.
#pragma once

#include <algorithm>
#include <functional>

#include "can_bus/bus.hpp"
#include "can_bus/robot_can.hpp"
#include "mdp_mission/mission.hpp"
#include "virtual_stm32/virtual_stm.hpp"

namespace mission {

class CanSimLink : public Link {
 public:
  CanSimLink(const mdp::Pose& start, const can::Bus::Config& bus_cfg = {},
             const stm::FaultConfig& device_faults = {}, const stm::BridgeConfig& bridge_cfg = {})
      : bus_(bus_cfg),
        host_(bus_.add_node("host")),
        ecu_(bus_.add_node("motor_ecu")),
        stm_(start, device_faults),
        bridge_(start, bridge_cfg) {}

  stm::BridgeCore& bridge() override { return bridge_; }
  double now_s() const override { return t_; }
  void advance(double dt_s) override {
    double remaining = dt_s;
    while (remaining > 1e-12) {
      const double dt = std::min(remaining, 0.01);
      host_send_commands();
      bus_.advance_to(t_);
      device_receive_commands();
      stm_.step(dt);
      device_send_output();
      t_ += dt;
      bus_.advance_to(t_);
      host_receive();
      bridge_.poll(t_);
      if (observer_) observer_(t_, stm_.true_pose(), bridge_.odometry().pose);
      remaining -= dt;
    }
  }

  // Called after every 10 ms step with (time, true pose, odometry pose).
  using Observer = std::function<void(double, const mdp::Pose&, const mdp::Pose&)>;
  void set_observer(Observer o) { observer_ = std::move(o); }

  can::Bus& bus() { return bus_; }
  can::Node& host_node() { return host_; }
  can::Node& ecu_node() { return ecu_; }
  stm::VirtualStm& device() { return stm_; }
  const can::HostDecoder& decoder() const { return decoder_; }

 private:
  void host_send_commands() {
    // The driver speaks text lines; turn each into a CAN command frame.
    framer_host_tx_ += bridge_.take_tx();
    std::size_t nl;
    while ((nl = framer_host_tx_.find('\n')) != std::string::npos) {
      const std::string line = framer_host_tx_.substr(0, nl);
      framer_host_tx_.erase(0, nl + 1);
      if (const auto cmd = stm::parse_command(line)) host_.send(can::command_to_frame(*cmd));
    }
  }

  void device_receive_commands() {
    while (const auto frame = ecu_.receive()) {
      if (const auto line = can::frame_to_command_line(*frame)) stm_.receive(*line + "\n");
    }
  }

  void device_send_output() {
    for (const auto& line : device_framer_.feed(stm_.take_output())) {
      for (const auto& frame : can::line_to_frames(line)) ecu_.send(frame);
    }
  }

  void host_receive() {
    while (const auto frame = host_.receive()) {
      for (const auto& line : decoder_.feed(*frame)) bridge_.on_bytes(line + "\n", t_);
    }
  }

  can::Bus bus_;
  can::Node& host_;
  can::Node& ecu_;
  stm::VirtualStm stm_;
  stm::BridgeCore bridge_;
  can::HostDecoder decoder_;
  stm::LineFramer device_framer_{stm::kMaxHostLineChars};
  std::string framer_host_tx_;
  double t_{0.0};
  Observer observer_;
};

}  // namespace mission
