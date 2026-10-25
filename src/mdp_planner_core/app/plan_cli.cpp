// Command-line driver used for benchmarking and parity checks against the Python planner.
//
// Input (text, one record per line):
//   case <name>
//   start <x> <y> <heading_rad>
//   obs <id> <x> <y> <face>
//   leg <sx> <sy> <sh> <tx> <ty> <th>      plan one leg between two exact poses
//   targets                                 print the viewing pose of every obstacle
//   route <time_limit_ms>                   plan the whole route over the case's obstacles
//   end
// Output: one JSON object per leg/route, doubles printed with 17 significant digits.
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "mdp_planner_core/route.hpp"

namespace {

using namespace mdp;

struct Case {
  std::string name;
  Pose start;
  std::vector<Obstacle> obstacles;
};

long micros_since(Clock::time_point t0) {
  return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t0).count();
}

void print_commands(const LatticePlan& plan) {
  std::printf("[");
  for (std::size_t i = 0; i < plan.actions.size(); ++i) {
    std::printf("%s\"%s\"", i ? "," : "", wire_string(plan.actions[i].cmd, plan.actions[i].value).c_str());
  }
  std::printf("]");
}

void run_leg(const Case& c, int idx, const Pose& from, const Pose& to, const PlannerConfig& cfg) {
  const auto t0 = Clock::now();
  const LatticePlan raw = plan_lattice_path(from, to, c.obstacles, cfg);
  const long search_us = micros_since(t0);
  LatticePlan plan = raw;
  try {
    plan = compress_lattice_plan(raw, c.obstacles, cfg);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "compression error: %s\n", e.what());
  }
  const long total_us = micros_since(t0);
  std::printf("{\"case\":\"%s\",\"kind\":\"leg\",\"idx\":%d,\"valid\":%s,\"reason\":\"%s\","
              "\"cost\":%.17g,\"distance\":%.17g,\"expansions\":%ld,\"final\":[%.17g,%.17g,%.17g],"
              "\"search_us\":%ld,\"time_us\":%ld,\"cmds\":",
              c.name.c_str(), idx, plan.valid ? "true" : "false", reason_name(plan.reason),
              plan.total_cost, plan.total_distance_cm, plan.expansions, plan.final_pose.x,
              plan.final_pose.y, plan.final_pose.heading, search_us, total_us);
  print_commands(plan);
  std::printf("}\n");
  std::fflush(stdout);
}

void run_route(const Case& c, long limit_ms, const PlannerConfig& cfg) {
  RouteOptions opts;
  opts.time_limit = std::chrono::milliseconds(limit_ms);
  const LegPlanner planner = default_leg_planner(c.obstacles, cfg);
  const auto t0 = Clock::now();
  const RouteResult r = optimise_route(c.start, c.obstacles, cfg, planner, opts);
  const long us = micros_since(t0);
  std::printf("{\"case\":\"%s\",\"kind\":\"route\",\"valid\":%s,\"complete\":%s,\"timed_out\":%s,"
              "\"cost\":%.17g,\"order\":[",
              c.name.c_str(), r.valid ? "true" : "false", r.complete ? "true" : "false",
              r.timed_out ? "true" : "false", r.total_cost);
  for (std::size_t i = 0; i < r.legs.size(); ++i) std::printf("%s%d", i ? "," : "", r.legs[i].obstacle_id);
  std::printf("],\"leg_requests\":%ld,\"leg_plans\":%ld,\"bound_prunes\":%ld,\"failed_leg_plans\":%ld,"
              "\"leg_time_ms\":%.1f,\"failed_leg_time_ms\":%.1f,\"time_us\":%ld,\"cmds\":[",
              r.stats.leg_requests, r.stats.leg_plans, r.stats.bound_prunes, r.stats.failed_leg_plans,
              r.stats.leg_time_ms, r.stats.failed_leg_time_ms, us);
  bool first = true;
  for (const auto& leg : r.legs) {
    for (const auto& a : leg.plan.actions) {
      std::printf("%s\"%s\"", first ? "" : ",", wire_string(a.cmd, a.value).c_str());
      first = false;
    }
  }
  std::printf("]}\n");
  std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: mdp_plan_cli <cases.txt>\n");
    return 2;
  }
  std::ifstream in(argv[1]);
  if (!in) {
    std::fprintf(stderr, "cannot open %s\n", argv[1]);
    return 2;
  }
  const PlannerConfig cfg;
  Case current;
  int leg_idx = 0;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string tag;
    ss >> tag;
    if (tag == "case") {
      current = Case{};
      leg_idx = 0;
      ss >> current.name;
    } else if (tag == "start") {
      ss >> current.start.x >> current.start.y >> current.start.heading;
    } else if (tag == "obs") {
      Obstacle o;
      ss >> o.id >> o.x >> o.y >> o.face;
      current.obstacles.push_back(o);
    } else if (tag == "leg") {
      Pose a, b;
      ss >> a.x >> a.y >> a.heading >> b.x >> b.y >> b.heading;
      run_leg(current, leg_idx++, a, b, cfg);
    } else if (tag == "targets") {
      std::printf("{\"case\":\"%s\",\"kind\":\"targets\",\"targets\":[", current.name.c_str());
      for (std::size_t i = 0; i < current.obstacles.size(); ++i) {
        const Pose t = viewing_target(current.obstacles[i], cfg);
        std::printf("%s[%.17g,%.17g,%.17g]", i ? "," : "", t.x, t.y, t.heading);
      }
      std::printf("]}\n");
    } else if (tag == "route") {
      long ms = 70000;
      ss >> ms;
      run_route(current, ms, cfg);
    }
  }
  return 0;
}
