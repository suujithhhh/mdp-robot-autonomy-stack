// ROS 2 wrapper around the C++ route planner.
//
// NOT YET RUN: compiles on ROS 2 Humble in CI, not launched. The planner itself is covered by the
// mdp_planner_core_tests suite and compared against the Python planner with tools/compare.py.
//
// Parameters   obstacles ("x,y,face;x,y,face;..." in cm), start_x_cm, start_y_cm, start_yaw_deg,
//              time_limit_s
// Service      /plan (std_srvs/Trigger): plan with the current parameters
// Topics       /planned_path  nav_msgs/Path                    (metres, frame "map")
//              /obstacles     visualization_msgs/MarkerArray   (obstacles + viewing poses)
//              /planned_commands std_msgs/String               ("MOV,F,30\nMOV,FL,90\n...")
// The latter three are latched (transient_local) so RViz and the mission node can start later.
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mdp_planner_core/route.hpp"
#include "mdp_planner_core/swept.hpp"

namespace {

std::vector<mdp::Obstacle> parse_obstacles(const std::string& spec) {
  std::vector<mdp::Obstacle> out;
  std::stringstream ss(spec);
  std::string item;
  int id = 1;
  while (std::getline(ss, item, ';')) {
    double x = 0, y = 0;
    char face = 'N';
    if (std::sscanf(item.c_str(), "%lf,%lf,%c", &x, &y, &face) == 3)
      out.push_back({id++, x, y, face});
  }
  return out;
}

geometry_msgs::msg::PoseStamped to_pose(const mdp::Pose& p, const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = stamp;
  ps.header.frame_id = "map";
  ps.pose.position.x = p.x / 100.0;
  ps.pose.position.y = p.y / 100.0;
  ps.pose.orientation.z = std::sin(p.heading / 2.0);
  ps.pose.orientation.w = std::cos(p.heading / 2.0);
  return ps;
}

}  // namespace

class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode() : Node("mdp_planner") {
    declare_parameter("obstacles", std::string("30,100,S;100,20,W;130,50,W"));
    declare_parameter("start_x_cm", 20.0);
    declare_parameter("start_y_cm", 10.0);
    declare_parameter("start_yaw_deg", 90.0);
    declare_parameter("time_limit_s", 70.0);

    const auto latched = rclcpp::QoS(1).transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", latched);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles", latched);
    commands_pub_ = create_publisher<std_msgs::msg::String>("/planned_commands", latched);
    service_ = create_service<std_srvs::srv::Trigger>(
        "/plan", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) { plan(*res); });
  }

 private:
  void plan(std_srvs::srv::Trigger::Response& res) {
    const mdp::PlannerConfig cfg;
    const auto obstacles = parse_obstacles(get_parameter("obstacles").as_string());
    const mdp::Pose start{get_parameter("start_x_cm").as_double(),
                          get_parameter("start_y_cm").as_double(),
                          get_parameter("start_yaw_deg").as_double() * mdp::kDegToRad};
    mdp::RouteOptions opts;
    opts.time_limit = std::chrono::milliseconds(
        static_cast<long>(get_parameter("time_limit_s").as_double() * 1000));

    const auto t0 = std::chrono::steady_clock::now();
    const mdp::RouteResult route =
        mdp::optimise_route(start, obstacles, cfg, mdp::default_leg_planner(obstacles, cfg), opts);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!route.valid) {
      res.success = false;
      res.message = "no route: " + route.reason;
      return;
    }

    const auto stamp = now();
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "map";
    path.poses.push_back(to_pose(start, stamp));
    std::string commands;
    mdp::Pose cursor = start;
    for (const auto& leg : route.legs) {
      for (const auto& a : leg.plan.actions) {
        for (const auto& p : mdp::sample_primitive(cursor, a.cmd, a.value, cfg).poses)
          path.poses.push_back(to_pose(p, stamp));
        cursor = mdp::propagate(cursor, a.cmd, a.value, cfg);
        commands +=
            "MOV," + std::string(mdp::cmd_name(a.cmd)) + "," + std::to_string(a.value) + "\n";
      }
    }
    path_pub_->publish(path);
    std_msgs::msg::String cmds;
    cmds.data = commands;
    commands_pub_->publish(cmds);
    publish_markers(obstacles, route, cfg, stamp);

    res.success = true;
    res.message = "planned " + std::to_string(route.legs.size()) + "/" +
                  std::to_string(obstacles.size()) + " obstacles in " + std::to_string(ms) + " ms";
    RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  }

  void publish_markers(const std::vector<mdp::Obstacle>& obstacles, const mdp::RouteResult& route,
                       const mdp::PlannerConfig& cfg, const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    for (const auto& o : obstacles) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = "map";
      m.ns = "obstacles";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.pose.position.x = (o.x + cfg.obstacle_cm / 2) / 100.0;
      m.pose.position.y = (o.y + cfg.obstacle_cm / 2) / 100.0;
      m.pose.position.z = 0.05;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = cfg.obstacle_cm / 100.0;
      m.scale.z = 0.1;
      m.color.r = 0.8f;
      m.color.g = 0.2f;
      m.color.b = 0.2f;
      m.color.a = 1.0f;
      array.markers.push_back(m);
    }
    for (const auto& leg : route.legs) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = "map";
      m.ns = "viewing_poses";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.pose = to_pose(leg.target, stamp).pose;
      m.scale.x = 0.15;
      m.scale.y = m.scale.z = 0.03;
      m.color.g = 0.8f;
      m.color.a = 1.0f;
      array.markers.push_back(m);
    }
    markers_pub_->publish(array);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commands_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
