// Mission sequencer: sends the planned commands to the driver one at a time and advances only
// when the previous one is ACKed. Any other outcome aborts the mission (PROTOCOL.md section 5).
// It never resends a command.
//
// NOT YET RUN: compiles on ROS 2 Humble in CI, not launched. The same policy, with retry handling,
// is implemented and tested in mission::run_mission (src/mdp_mission) against the virtual STM32.
//
// Service /run_mission (std_srvs/Trigger); topics /planned_commands (in), /stm32/result (in),
// /stm32/command (out), /mission/status (out).
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class MissionNode : public rclcpp::Node {
 public:
  MissionNode() : Node("mission") {
    cmd_pub_ = create_publisher<std_msgs::msg::String>("/stm32/command", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/mission/status", 10);
    plan_sub_ = create_subscription<std_msgs::msg::String>(
        "/planned_commands", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) { load(msg->data); });
    result_sub_ = create_subscription<std_msgs::msg::String>(
        "/stm32/result", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_result(msg->data); });
    service_ = create_service<std_srvs::srv::Trigger>(
        "/run_mission", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          if (commands_.empty()) {
            res->success = false;
            res->message = "no planned commands yet: call /plan first";
          } else if (running_) {
            res->success = false;
            res->message = "mission already running";
          } else {
            index_ = 0;
            running_ = true;
            send_current();
            res->success = true;
            res->message = "started " + std::to_string(commands_.size()) + " commands";
          }
        });
  }

 private:
  void load(const std::string& text) {
    commands_.clear();
    std::stringstream ss(text);
    std::string line;
    while (std::getline(ss, line))
      if (!line.empty()) commands_.push_back(line);
  }

  void status(const std::string& text) {
    std_msgs::msg::String m;
    m.data = text;
    status_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void send_current() {
    std_msgs::msg::String m;
    m.data = commands_[index_];
    cmd_pub_->publish(m);
  }

  void on_result(const std::string& text) {
    if (!running_ || index_ >= commands_.size()) return;
    // "<wire command> <OUTCOME>": ignore results of other commands (e.g. the start-up IMU,GET).
    const std::string& expect = commands_[index_];
    if (text.rfind(expect + " ", 0) != 0) return;
    const std::string outcome = text.substr(expect.size() + 1);
    if (outcome != "ACK") {
      running_ = false;
      status("ABORTED at " + expect + ": " + outcome);
      return;
    }
    if (++index_ >= commands_.size()) {
      running_ = false;
      status("COMPLETED " + std::to_string(commands_.size()) + " commands");
      return;
    }
    send_current();
  }

  std::vector<std::string> commands_;
  std::size_t index_{0};
  bool running_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_, status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_sub_, result_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
