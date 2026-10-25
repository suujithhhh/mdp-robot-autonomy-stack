#include "mdp_planner_core/route.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>

#include "mdp_planner_core/footprint.hpp"

namespace mdp {

Pose viewing_target(const Obstacle& obstacle, const PlannerConfig& cfg) {
  double out_x = 0.0, out_y = 0.0, heading = 0.0;
  switch (obstacle.face) {
    case 'N': out_x = 0.0; out_y = 1.0; heading = -kPi / 2; break;
    case 'E': out_x = 1.0; out_y = 0.0; heading = kPi; break;
    case 'S': out_x = 0.0; out_y = -1.0; heading = kPi / 2; break;
    case 'W': out_x = -1.0; out_y = 0.0; heading = 0.0; break;
    default: throw std::invalid_argument("obstacle face must be one of N, E, S, W");
  }
  const double half = cfg.obstacle_cm / 2;
  const double lens_x = obstacle.x + half + out_x * (half + cfg.camera_standoff_cm);
  const double lens_y = obstacle.y + half + out_y * (half + cfg.camera_standoff_cm);
  // The robot faces into the image face: forward = -outward, left = forward rotated +90 deg.
  const double forward_x = -out_x, forward_y = -out_y;
  const double left_x = -forward_y, left_y = forward_x;
  return {lens_x - cfg.camera_forward_offset_cm * forward_x - cfg.camera_lateral_offset_cm * left_x,
          lens_y - cfg.camera_forward_offset_cm * forward_y - cfg.camera_lateral_offset_cm * left_y,
          heading};
}

LegPlanner default_leg_planner(const std::vector<Obstacle>& obstacles, const PlannerConfig& cfg) {
  return [obstacles, cfg](const Pose& from, const Pose& to, Clock::time_point deadline) {
    const LatticePlan raw = plan_lattice_path(from, to, obstacles, cfg, deadline);
    return compress_lattice_plan(raw, obstacles, cfg);
  };
}

// ---------------------------------------------------------------- Held-Karp lower bound

HeldKarpBound::HeldKarpBound(const std::vector<Pose>& targets, const PlannerConfig& cfg)
    : targets_(targets), cfg_(cfg) {
  const int n = static_cast<int>(targets.size());
  if (n > 16) throw std::invalid_argument("HeldKarpBound supports at most 16 targets");
  const unsigned full = (1u << n);
  path_.assign(n, std::vector<double>(full, std::numeric_limits<double>::infinity()));
  for (int j = 0; j < n; ++j) path_[j][0] = 0.0;
  // Increasing mask order guarantees S \ {k} is already solved when S is.
  for (unsigned s = 1; s < full; ++s) {
    for (int j = 0; j < n; ++j) {
      if (s & (1u << j)) continue;  // j must not be in the set it still has to visit
      double best = std::numeric_limits<double>::infinity();
      for (int k = 0; k < n; ++k) {
        if (!(s & (1u << k))) continue;
        best = std::min(best, edge(j, k) + path_[k][s & ~(1u << k)]);
      }
      path_[j][s] = best;
    }
  }
}

// A leg ends within the goal tolerance of target a and must reach within tolerance of target b,
// so its length is at least dist(a, b) - 2 * tolerance. Cost >= length * min(1, reverse_penalty).
double HeldKarpBound::edge(int a, int b) const {
  const double d = std::hypot(targets_[a].x - targets_[b].x, targets_[a].y - targets_[b].y);
  return std::max(0.0, d - 2.0 * cfg_.lattice_goal_position_tolerance_cm) *
         std::min(1.0, cfg_.reverse_penalty);
}

double HeldKarpBound::lower_bound(const Pose& from, unsigned remaining) const {
  if (remaining == 0) return 0.0;
  const int n = static_cast<int>(targets_.size());
  double best = std::numeric_limits<double>::infinity();
  for (int j = 0; j < n; ++j) {
    if (!(remaining & (1u << j))) continue;
    const double d = std::hypot(from.x - targets_[j].x, from.y - targets_[j].y);
    const double first = std::max(0.0, d - cfg_.lattice_goal_position_tolerance_cm) *
                         std::min(1.0, cfg_.reverse_penalty);
    best = std::min(best, first + path_[j][remaining & ~(1u << j)]);
  }
  return best;
}

// ---------------------------------------------------------------- route search

namespace {

using LegKey = std::array<std::uint64_t, 6>;

std::uint64_t bits(double v) {
  std::uint64_t u;
  std::memcpy(&u, &v, sizeof u);
  return u;
}

LegKey leg_key(const Pose& a, const Pose& b) {
  return {bits(a.x), bits(a.y), bits(a.heading), bits(b.x), bits(b.y), bits(b.heading)};
}

class Search {
 public:
  Search(const Pose& start, const std::vector<Obstacle>& obstacles, const PlannerConfig& cfg,
         const LegPlanner& planner, const RouteOptions& opts)
      : start_(start), obstacles_(obstacles), cfg_(cfg), planner_(planner), opts_(opts) {}

  RouteResult run() {
    const int n = static_cast<int>(obstacles_.size());
    RouteResult result;
    if (n == 0) {
      result.valid = true;
      result.complete = true;
      return result;
    }
    if (n > 16) throw std::invalid_argument("at most 16 obstacles are supported");

    // A target whose footprint already hits an obstacle (or leaves the arena) can never be reached.
    for (const auto& o : obstacles_) {
      const Pose t = viewing_target(o, cfg_);
      if (!pose_is_safe(t, obstacles_, cfg_)) {
        result.reason = "no_safe_viewing_pose";
        return result;
      }
      targets_.push_back(t);
    }
    if (opts_.use_held_karp_bound) bound_ = std::make_unique<HeldKarpBound>(targets_, cfg_);

    deadline_ = Clock::now() + opts_.time_limit;
    full_mask_ = (n >= 32) ? ~0u : ((1u << n) - 1u);
    std::vector<RouteLeg> chain;
    dfs(start_, 0u, 0.0, chain);

    result.stats = stats_;
    result.timed_out = timed_out_;
    if (have_best_) {
      result.valid = true;
      result.complete = best_complete_;
      result.legs = best_chain_;
      result.total_cost = best_cost_;
    } else {
      result.reason = timed_out_ ? "timeout" : "no_route";
    }
    return result;
  }

 private:
  const LatticePlan& plan_leg(const Pose& from, const Pose& to) {
    ++stats_.leg_requests;
    const LegKey key = leg_key(from, to);
    const auto it = cache_.find(key);
    if (it != cache_.end()) return it->second;
    ++stats_.leg_plans;
    const auto leg_t0 = Clock::now();
    LatticePlan plan = planner_(from, to, deadline_);
    const double leg_ms = std::chrono::duration<double, std::milli>(Clock::now() - leg_t0).count();
    stats_.leg_time_ms += leg_ms;
    if (!plan.valid) {
      ++stats_.failed_leg_plans;
      stats_.failed_leg_time_ms += leg_ms;
    }
    if (plan.reason == FailReason::Timeout) {
      timed_out_ = true;
      return plan_scratch_ = std::move(plan);  // never cache a timeout: it is not a real answer
    }
    return cache_.emplace(key, std::move(plan)).first->second;
  }

  // Remember the best verified route so far: more obstacles first, then lower cost.
  void record(const std::vector<RouteLeg>& chain, double cost, bool complete) {
    if (complete) {
      if (best_complete_ && cost >= best_cost_) return;
      best_complete_ = true;
      ++stats_.incumbent_updates;
    } else {
      if (best_complete_) return;
      if (have_best_ && (chain.size() < best_chain_.size() ||
                         (chain.size() == best_chain_.size() && cost >= best_cost_)))
        return;
    }
    have_best_ = true;
    best_chain_ = chain;
    best_cost_ = cost;
  }

  void dfs(const Pose& current, unsigned visited, double g, std::vector<RouteLeg>& chain) {
    if (timed_out_) return;
    ++stats_.nodes_visited;
    if (!chain.empty()) record(chain, g, visited == full_mask_);
    if (visited == full_mask_) return;
    if (Clock::now() >= deadline_) {
      timed_out_ = true;
      return;
    }

    const unsigned remaining = full_mask_ & ~visited;
    // Prune this whole subtree if even the optimistic completion cannot beat the incumbent.
    if (best_complete_ && bound_ &&
        g + bound_->lower_bound(current, remaining) >= best_cost_) {
      ++stats_.bound_prunes;
      return;
    }

    // Try nearer targets first so a good incumbent appears early and prunes more.
    std::vector<std::pair<double, int>> order;
    for (int i = 0; i < static_cast<int>(targets_.size()); ++i) {
      if (visited & (1u << i)) continue;
      order.emplace_back(std::hypot(current.x - targets_[i].x, current.y - targets_[i].y), i);
    }
    std::sort(order.begin(), order.end());

    for (const auto& entry : order) {
      const int i = entry.second;
      if (best_complete_ && g >= best_cost_) break;
      const LatticePlan& leg = plan_leg(current, targets_[i]);
      if (timed_out_) return;
      if (!leg.valid) continue;  // unreachable from here: prunes only this prefix
      const double new_g = g + leg.total_cost;
      if (best_complete_ && new_g >= best_cost_) {
        ++stats_.bound_prunes;
        continue;
      }
      chain.push_back({obstacles_[i].id, targets_[i], leg});
      dfs(leg.final_pose, visited | (1u << i), new_g, chain);
      chain.pop_back();
      if (timed_out_) return;
    }
  }

  Pose start_;
  const std::vector<Obstacle>& obstacles_;
  PlannerConfig cfg_;
  const LegPlanner& planner_;
  RouteOptions opts_;

  std::vector<Pose> targets_;
  std::unique_ptr<HeldKarpBound> bound_;
  std::map<LegKey, LatticePlan> cache_;
  LatticePlan plan_scratch_;
  Clock::time_point deadline_;
  unsigned full_mask_{0};
  bool timed_out_{false};
  bool have_best_{false};
  bool best_complete_{false};
  double best_cost_{0.0};
  std::vector<RouteLeg> best_chain_;
  RouteStats stats_;
};

}  // namespace

RouteResult optimise_route(const Pose& start, const std::vector<Obstacle>& obstacles,
                           const PlannerConfig& cfg, const LegPlanner& planner,
                           const RouteOptions& opts) {
  Search search(start, obstacles, cfg, planner, opts);
  return search.run();
}

}  // namespace mdp
