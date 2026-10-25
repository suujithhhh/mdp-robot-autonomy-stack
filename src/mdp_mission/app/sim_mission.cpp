// Plan a route with the C++ planner, then drive it through the driver core against the virtual
// STM32 in simulated time. Prints a summary and can record the paths for plotting.
//
//   mdp_sim_mission [--obs "x,y,face;x,y,face;..."] [--start x,y,heading_deg] [--csv path.csv]
//                   [--slip 0.05] [--noise 0.01]
// The default layout is the A_spread_03 fixture of the MDP planner.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "mdp_mission/sim_link.hpp"

namespace {

std::vector<mdp::Obstacle> parse_obstacles(const std::string& spec) {
  std::vector<mdp::Obstacle> out;
  std::stringstream ss(spec);
  std::string item;
  int id = 1;
  while (std::getline(ss, item, ';')) {
    double x = 0, y = 0;
    char face = 'N';
    if (std::sscanf(item.c_str(), "%lf,%lf,%c", &x, &y, &face) == 3) out.push_back({id++, x, y, face});
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<mdp::Obstacle> obstacles = {{1, 30, 100, 'S'}, {2, 100, 20, 'W'}, {3, 130, 50, 'W'}};
  mdp::Pose start{20, 10, mdp::kPi / 2};
  std::string csv_path;
  stm::FaultConfig faults;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    if (key == "--obs") obstacles = parse_obstacles(argv[i + 1]);
    else if (key == "--csv") csv_path = argv[i + 1];
    else if (key == "--slip") faults.slip = std::atof(argv[i + 1]);
    else if (key == "--noise") faults.encoder_noise = std::atof(argv[i + 1]);
    else if (key == "--start") {
      double x, y, h;
      if (std::sscanf(argv[i + 1], "%lf,%lf,%lf", &x, &y, &h) == 3) start = {x, y, h * mdp::kDegToRad};
    }
  }

  const mdp::PlannerConfig cfg;
  mdp::RouteOptions opts;
  opts.time_limit = std::chrono::seconds(70);
  const auto t0 = std::chrono::steady_clock::now();
  const mdp::RouteResult route =
      mdp::optimise_route(start, obstacles, cfg, mdp::default_leg_planner(obstacles, cfg), opts);
  const double plan_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  std::printf("planned %zu/%zu obstacles in %.1f ms  (complete=%d timed_out=%d cost=%.1f cm-equiv)\n",
              route.legs.size(), obstacles.size(), plan_ms, route.complete, route.timed_out,
              route.total_cost);
  std::printf("  leg plans %ld, cache hits %ld, bound prunes %ld\n", route.stats.leg_plans,
              route.stats.leg_requests - route.stats.leg_plans, route.stats.bound_prunes);
  if (!route.valid) {
    std::printf("no route: %s\n", route.reason.c_str());
    return 1;
  }

  std::ofstream csv;
  if (!csv_path.empty()) csv.open(csv_path);
  if (csv) csv << "t,true_x,true_y,true_h,odom_x,odom_y,odom_h\n";

  mission::SimLink link(start, faults);
  link.set_observer([&](double t, const mdp::Pose& truth, const mdp::Pose& odom) {
    static long n = 0;
    if (csv && (n++ % 10) == 0)
      csv << t << ',' << truth.x << ',' << truth.y << ',' << truth.heading << ',' << odom.x << ','
          << odom.y << ',' << odom.heading << '\n';
  });
  link.bridge().send_imu_get(0.0);
  link.advance(0.1);

  const mission::MissionResult r = mission::run_mission(route, link);
  std::printf("mission: %s  commands %zu/%zu  simulated time %.1f s  pose_uncertain=%d\n",
              r.completed ? "COMPLETED" : "ABORTED", r.commands_done, r.commands_total, r.elapsed_s,
              r.pose_uncertain);
  if (!r.completed)
    std::printf("  failure: %s on %s\n", stm::outcome_name(r.failure), r.failed_wire.c_str());
  for (const auto& s : r.snapshots) {
    std::printf("  obstacle %d: target (%.1f, %.1f)  commanded error %.2f cm  odometry error %.2f cm\n",
                s.obstacle_id, s.target.x, s.target.y,
                std::hypot(s.commanded.x - s.target.x, s.commanded.y - s.target.y),
                std::hypot(s.odometry.x - s.target.x, s.odometry.y - s.target.y));
  }
  const mdp::Pose truth = link.device().true_pose();
  std::printf("final: true (%.1f, %.1f)  commanded (%.1f, %.1f)  odometry (%.1f, %.1f)\n", truth.x,
              truth.y, link.bridge().commanded_pose().x, link.bridge().commanded_pose().y,
              link.bridge().odometry().pose.x, link.bridge().odometry().pose.y);
  return r.completed ? 0 : 2;
}
