// C++ command mux: drop-in replacement for scripts/cmd_vel_mux.py (same topics, parameters, behaviour, log lines).
//   ros2 launch mobile_robot_bringup launch_sim.launch.py mux:=cpp
// The logic lives in include/mobile_robot_bringup/mux_core.hpp (tested against the Python mux with golden vectors); this file
// is only the ROS plumbing.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mobile_robot_bringup/mux_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CmdVelMux : public rclcpp::Node
{
public:
  CmdVelMux()
  : Node("cmd_vel_mux"), front_(28.0, 0.0), rear_(28.0, 180.0)
  {
    const auto out = declare_parameter<std::string>("output_topic", "/diff_cont/cmd_vel");
    stamped_ = declare_parameter<bool>("stamped", true);
    timeouts_ = {declare_parameter<double>("teleop_timeout", 0.6), declare_parameter<double>("nav_timeout", 0.5),
      declare_parameter<double>("auto_timeout", 0.5)};
    const auto odom_in = declare_parameter<std::string>("odom_in", "/diff_cont/odom");
    const auto odom_out = declare_parameter<std::string>("odom_out", "/odom");
    safety_ = declare_parameter<bool>("safety_enabled", true);
    p_.stop_dist = declare_parameter<double>("safety_stop_distance", 0.38);
    p_.slow_dist = declare_parameter<double>("safety_slow_distance", 1.0);
    const double cone = declare_parameter<double>("safety_cone_deg", 28.0);
    p_.rear_stop_dist = declare_parameter<double>("rear_stop_distance", 0.3);
    p_.require_scan = declare_parameter<bool>("require_scan", true);
    p_.max_linear = declare_parameter<double>("max_linear", 0.6);
    p_.max_angular = declare_parameter<double>("max_angular", 2.0);
    p_.max_accel = declare_parameter<double>("max_accel", 1.2);
    p_.max_ang_accel = declare_parameter<double>("max_ang_accel", 4.0);
    front_ = mux::ConeFinder(cone, 0.0);
    rear_ = mux::ConeFinder(cone, 180.0);

    if (stamped_) {
      pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>(out, 10);
    } else {
      pub_plain_ = create_publisher<geometry_msgs::msg::Twist>(out, 10);
    }
    const std::pair<const char *, mux::Source> inputs[] = {
      {"/cmd_vel_teleop", mux::kTeleop}, {"/cmd_vel", mux::kNav}, {"/cmd_vel_nav", mux::kNav},
      {"/cmd_vel_smoothed", mux::kNav}, {"/cmd_vel_auto", mux::kAuto}};
    for (const auto & in : inputs) {
      const mux::Source src = in.second;
      cmd_subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(
          in.first, 10, [this, src](geometry_msgs::msg::Twist::ConstSharedPtr m) {
            last_[src] = {true, now_s(), {m->linear.x, m->angular.z}};
          }));
    }
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {on_scan(*m);});
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", rclcpp::QoS(1).reliable().transient_local(), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data != estop_) {
          estop_ = m->data;
          if (estop_) {
            RCLCPP_WARN(get_logger(), "EMERGENCY STOP ENGAGED: robot stopped until /emergency_stop is set to false");
          } else {
            RCLCPP_WARN(get_logger(), "EMERGENCY STOP released");
          }
        }
      });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_out, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_in, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {odom_pub_->publish(*m);});
    status_pub_ = create_publisher<std_msgs::msg::String>("/mux_status", 10);
    tick_timer_ = create_wall_timer(50ms, [this]() {tick();});
    status_timer_ = create_wall_timer(500ms, [this]() {publish_status();});
    RCLCPP_INFO(
      get_logger(), "cmd_vel_mux (C++): teleop > nav > auto -> %s (%s), safety %s (stop %.2f m, slow %.2f m)", out.c_str(),
      stamped_ ? "TwistStamped" : "Twist", safety_ ? "ON" : "OFF", p_.stop_dist, p_.slow_dist);
  }

private:
  static double now_s()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void on_scan(const sensor_msgs::msg::LaserScan & m)
  {
    const double rmax = m.range_max > 0.0f ? static_cast<double>(m.range_max) : 12.0;
    front_distance_ = front_(m.ranges.data(), m.ranges.size(), m.angle_min, m.angle_increment, rmax);
    rear_distance_ = rear_(m.ranges.data(), m.ranges.size(), m.angle_min, m.angle_increment, rmax);
    scan_time_ = now_s();
  }

  void tick()
  {
    const double now = now_s();
    const auto sel = mux::select(last_, now, timeouts_);
    if (sel.first != current_) {
      current_ = sel.first;
      RCLCPP_INFO(get_logger(), "driving source: %s", source_name(current_));
    }
    mux::Command target = sel.second;
    std::string reason;
    bool brake = false;
    if (estop_) {
      target = {0.0, 0.0};
      reason = "e-stop";
      brake = true;
    } else if (safety_) {
      const auto g = mux::guard(target.lin, target.ang, front_distance_, now - scan_time_ < 1.0, p_, rear_distance_);
      target = {g.lin, g.ang};
      reason = g.reason;
      brake = mux::is_braking(g.reason);
    }
    reason_ = reason;
    if (reason != last_reason_) {
      last_reason_ = reason;
      if (reason == "obstacle" || reason == "obstacle-behind" || reason == "no-scan") {
        RCLCPP_INFO(get_logger(), "safety: forward motion blocked (%s; nearest in front %.2f m)", reason.c_str(), front_distance_);
      }
    }
    cur_ = mux::limit_step(cur_, target, 0.05, p_, brake);
    if (stamped_) {
      geometry_msgs::msg::TwistStamped m;        // stamp 0 = "now" for the controller
      m.header.frame_id = "base_link";
      m.twist.linear.x = cur_.lin;
      m.twist.angular.z = cur_.ang;
      pub_stamped_->publish(m);
    } else {
      geometry_msgs::msg::Twist m;
      m.linear.x = cur_.lin;
      m.angular.z = cur_.ang;
      pub_plain_->publish(m);
    }
  }

  void publish_status()
  {
    std_msgs::msg::String s;
    std::string who = estop_ ? "ESTOP" : (current_ == mux::kNone ? "idle" : source_name(current_));
    s.data = who + (reason_.empty() ? "" : " " + reason_);
    status_pub_->publish(s);
  }

  static const char * source_name(mux::Source s)
  {
    switch (s) {
      case mux::kTeleop: return "teleop";
      case mux::kNav: return "nav";
      case mux::kAuto: return "auto";
      default: return "none (stopped)";
    }
  }

  bool stamped_ = true, safety_ = true, estop_ = false;
  mux::Params p_;
  std::array<double, 3> timeouts_{};
  std::array<mux::Received, 3> last_{};
  mux::ConeFinder front_, rear_;
  mux::Command cur_;
  mux::Source current_ = mux::kNone;
  double front_distance_ = 12.0, rear_distance_ = 12.0, scan_time_ = -100.0;
  std::string reason_, last_reason_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_stamped_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_plain_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> cmd_subs_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_, status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelMux>());
  rclcpp::shutdown();
  return 0;
}
