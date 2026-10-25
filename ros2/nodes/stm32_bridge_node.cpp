// ROS 2 serial driver for the STM32 (or for virtual_stm32_pty). A thin wrapper: all protocol
// logic lives in stm32::BridgeCore, which is unit-tested without ROS.
//
// UNVERIFIED: this file has not been compiled yet (it needs Ubuntu + ROS 2 Humble); the core it
// wraps is covered by the stm32_bridge_tests and mdp_mission_tests suites.
//
// Topics
//   sub  /stm32/command   std_msgs/String   "MOV,F,30" | "STOP" | "IMU,GET"
//   pub  /stm32/result    std_msgs/String   "<command> <OUTCOME>"
//   pub  /odom            nav_msgs/Odometry (metres; odom -> base_link)
//   pub  /imu/heading     sensor_msgs/Imu   (yaw only)
//   pub  /diagnostics     diagnostic_msgs/DiagnosticArray
//   tf   odom -> base_link
#include <fcntl.h>
#include <termios.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>

#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include "stm32_bridge/bridge_core.hpp"

namespace {

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace

class Stm32BridgeNode : public rclcpp::Node {
 public:
  Stm32BridgeNode()
      : Node("stm32_bridge"),
        core_(mdp::Pose{declare_parameter("initial_x_cm", 20.0),
                        declare_parameter("initial_y_cm", 10.0),
                        declare_parameter("initial_yaw_deg", 90.0) * mdp::kDegToRad}) {
    const std::string port = declare_parameter("port", std::string("/tmp/vstm32"));
    open_serial(port);

    result_pub_ = create_publisher<std_msgs::msg::String>("/stm32/result", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/heading", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "/stm32/command", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_command(msg->data); });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { tick(); });
    core_.send_imu_get(now_s());  // reference the IMU while the robot stands still
  }

  ~Stm32BridgeNode() override {
    if (fd_ >= 0) close(fd_);
  }

 private:
  double now_s() { return this->now().seconds(); }

  void open_serial(const std::string& port) {
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "cannot open %s", port.c_str());
      return;
    }
    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);  // 8N1, no echo, no line editing
    cfsetspeed(&tio, B115200);
    tcsetattr(fd_, TCSANOW, &tio);
  }

  void on_command(const std::string& text) {
    const auto cmd = stm::parse_command(text);
    if (!cmd) {
      RCLCPP_WARN(get_logger(), "ignoring malformed command '%s'", text.c_str());
      return;
    }
    if (const auto* m = std::get_if<stm::MoveCmd>(&*cmd)) {
      if (!core_.send_move(*m, now_s()))
        RCLCPP_WARN(get_logger(), "busy: '%s' refused", text.c_str());
    } else if (std::holds_alternative<stm::StopCmd>(*cmd)) {
      core_.send_stop(now_s());
    } else {
      core_.send_imu_get(now_s());
    }
  }

  void tick() {
    if (fd_ < 0) return;
    char buf[256];
    const ssize_t n = read(fd_, buf, sizeof buf);
    if (n > 0) core_.on_bytes(std::string_view(buf, static_cast<std::size_t>(n)), now_s());
    core_.poll(now_s());
    const std::string tx = core_.take_tx();
    if (!tx.empty() && write(fd_, tx.data(), tx.size()) < 0)
      RCLCPP_WARN(get_logger(), "serial write failed");

    while (const auto r = core_.take_result()) {
      std_msgs::msg::String msg;
      msg.data = r->wire + " " + stm::outcome_name(r->outcome);
      result_pub_->publish(msg);
    }
    publish_state();
  }

  void publish_state() {
    const auto stamp = this->now();
    const mdp::Pose& p = core_.odometry().pose;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = p.x / 100.0;  // the planner works in cm, ROS in metres
    odom.pose.pose.position.y = p.y / 100.0;
    odom.pose.pose.orientation = yaw_to_quaternion(p.heading);
    odom.twist.twist.linear.x = core_.odometry().v_cm_s / 100.0;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_->sendTransform(tf);

    if (const auto imu = core_.last_imu_deg()) {
      sensor_msgs::msg::Imu msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = "base_link";
      msg.orientation = yaw_to_quaternion(*imu * mdp::kDegToRad);
      imu_pub_->publish(msg);
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "stm32_link";
    status.level = core_.pose_uncertain() ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                          : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = core_.pose_uncertain() ? "pose uncertain" : "ok";
    auto kv = [&](const char* k, long v) {
      diagnostic_msgs::msg::KeyValue e;
      e.key = k;
      e.value = std::to_string(v);
      status.values.push_back(e);
    };
    kv("unparsed_lines", core_.unparsed_lines());
    kv("unexpected_replies", core_.unexpected_replies());
    kv("overflow_lines", core_.overflow_lines());
    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  int fd_{-1};
  stm::BridgeCore core_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stm32BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
