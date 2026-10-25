#include "mdp_mission/mission.hpp"

namespace mission {

FlatRoute flatten(const mdp::RouteResult& route) {
  FlatRoute flat;
  for (const auto& leg : route.legs) {
    for (const auto& a : leg.plan.actions) flat.commands.push_back({a.cmd, a.value});
    flat.leg_end_index.push_back(flat.commands.size());
  }
  return flat;
}

namespace {

// Let simulated time pass until the in-flight command has a result (or the cap expires).
std::optional<stm::CommandResult> wait_for_result(Link& link, const MissionOptions& opts) {
  const double give_up = link.now_s() + opts.max_wait_s;
  while (link.now_s() < give_up) {
    link.advance(opts.step_s);
    if (auto r = link.bridge().take_result()) return r;
  }
  return std::nullopt;
}

}  // namespace

MissionResult run_mission(const mdp::RouteResult& route, Link& link, const MissionOptions& opts) {
  MissionResult result;
  const FlatRoute flat = flatten(route);
  result.commands_total = flat.commands.size();
  const double t0 = link.now_s();
  stm::BridgeCore& bridge = link.bridge();

  std::size_t next_leg = 0;
  for (std::size_t i = 0; i < flat.commands.size(); ++i) {
    int attempts = 0;
    for (;;) {
      if (!bridge.send_move(flat.commands[i], link.now_s())) {
        result.failure = stm::Outcome::ErrBusy;  // local protocol violation: refuse to continue
        result.failed_wire = stm::encode_command(stm::Command{flat.commands[i]});
        result.elapsed_s = link.now_s() - t0;
        return result;
      }
      const auto r = wait_for_result(link, opts);
      if (r && r->outcome == stm::Outcome::Ack) break;

      const stm::Outcome why = r ? r->outcome : stm::Outcome::LinkTimeout;
      // Retry is allowed only when nothing moved: IMU_NOT_READY before the movement started.
      if (why == stm::Outcome::ErrImuNotReady && !bridge.pose_uncertain() &&
          attempts < opts.max_retries) {
        ++attempts;
        ++result.retries;
        link.advance(0.5);  // give the sensor a moment
        continue;
      }
      result.failure = why;
      result.failed_wire = r ? r->wire : stm::encode_command(stm::Command{flat.commands[i]});
      result.pose_uncertain = bridge.pose_uncertain();
      result.commands_done = i;
      result.elapsed_s = link.now_s() - t0;
      return result;
    }
    result.commands_done = i + 1;
    if (next_leg < flat.leg_end_index.size() && flat.leg_end_index[next_leg] == i + 1) {
      result.snapshots.push_back({route.legs[next_leg].obstacle_id, route.legs[next_leg].target,
                                  bridge.commanded_pose(), bridge.odometry().pose});
      ++next_leg;
    }
  }
  result.completed = true;
  result.pose_uncertain = bridge.pose_uncertain();
  result.elapsed_s = link.now_s() - t0;
  return result;
}

}  // namespace mission
