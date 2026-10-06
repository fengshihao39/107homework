#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_analysis/msg/odom_stats.hpp"
#include "tf2/LinearMath/Quaternion.h"

class OdomAnalysis : public rclcpp::Node {
 public:
  OdomAnalysis() : Node("odom_analysis") {
    const auto topic = declare_parameter<std::string>("input_topic", "/Odometry");
    expected_rate_ = positive("expected_rate_hz", 10.0);
    max_gap_ = positive("max_gap_seconds", 1.0);
    max_step_ = positive("max_step_m", 1.0);
    speed_limit_ = positive("max_speed_mps", 5.0);
    stats_pub_ = create_publisher<sensor_analysis::msg::OdomStats>("/analysis/odom_stats", 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/analysis/odom_velocity", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(topic, rclcpp::SensorDataQoS().keep_last(1000),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(*msg); });
    RCLCPP_INFO(get_logger(), "Odometry input %s; deriving world-frame velocity from poses", topic.c_str());
  }

 private:
  double positive(const std::string &name, double initial) {
    const auto value = declare_parameter<double>(name, initial);
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(name + " must be positive and finite");
    return value;
  }
  static double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
  }
  void reset() {
    have_last_ = false; timestamps_.clear();
    path_length_ = valid_duration_ = max_speed_ = 0.0;
    ++resets_;
  }
  void on_odom(const nav_msgs::msg::Odometry &input) {
    ++received_;
    const auto &p = input.pose.pose.position;
    const auto &q = input.pose.pose.orientation;
    const auto stamp = rclcpp::Time(input.header.stamp).nanoseconds();
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm) || norm < 1e-12) { ++rejected_; return; }
    tf2::Quaternion rotation(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
    if (have_last_ && (frame_ != input.header.frame_id || child_ != input.child_frame_id)) reset();
    if (have_last_ && stamp <= last_stamp_) {
      ++nonmonotonic_;
      if (stamp == last_stamp_) return;
      reset();
    }
    sensor_analysis::msg::OdomStats out;
    out.header = input.header; out.child_frame_id = input.child_frame_id;
    if (!have_last_) {
      first_stamp_ = stamp; first_position_ = p;
    } else {
      out.dt_seconds = (stamp - last_stamp_) / 1e9;
      const double step = distance(p, last_position_);
      const double speed = step / out.dt_seconds;
      if (out.dt_seconds > 2.5 / expected_rate_) ++gaps_;
      if (out.dt_seconds > max_gap_) {
        timestamps_.clear();
      } else if (!std::isfinite(speed) || step > max_step_ || speed > speed_limit_) {
        // Do not count a localization jump as physical travel.
        ++jumps_;
      } else {
        geometry_msgs::msg::TwistStamped velocity;
        velocity.header = input.header;
        velocity.twist.linear.x = (p.x - last_position_.x) / out.dt_seconds;
        velocity.twist.linear.y = (p.y - last_position_.y) / out.dt_seconds;
        velocity.twist.linear.z = (p.z - last_position_.z) / out.dt_seconds;
        // q_now * inverse(q_previous) expresses the rotation in the parent frame.
        auto delta = rotation * last_rotation_.inverse();
        delta.normalize();
        if (delta.w() < 0) delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
        const double axis_norm = std::hypot(delta.x(), delta.y(), delta.z());
        if (axis_norm > 1e-12) {
          const double factor = 2.0 * std::atan2(axis_norm, delta.w()) / (axis_norm * out.dt_seconds);
          velocity.twist.angular.x = delta.x() * factor;
          velocity.twist.angular.y = delta.y() * factor;
          velocity.twist.angular.z = delta.z() * factor;
        }
        out.velocity_valid = true; out.speed_mps = speed;
        const auto &angular = velocity.twist.angular;
        out.angular_speed_radps = std::hypot(angular.x, angular.y, angular.z);
        path_length_ += step; valid_duration_ += out.dt_seconds;
        max_speed_ = std::max(max_speed_, speed);
        velocity_pub_->publish(velocity);
      }
    }
    timestamps_.push_back(stamp);
    while (timestamps_.size() > 1 && stamp - timestamps_.front() > 1000000000LL) timestamps_.pop_front();
    const double span = (stamp - timestamps_.front()) / 1e9;
    out.sample_rate_hz = span > 0 ? (timestamps_.size() - 1) / span : 0.0;
    out.path_length_m = path_length_; out.displacement_m = distance(p, first_position_);
    out.elapsed_seconds = (stamp - first_stamp_) / 1e9;
    out.valid_duration_seconds = valid_duration_;
    out.average_speed_mps = valid_duration_ > 0 ? path_length_ / valid_duration_ : 0.0;
    out.max_speed_mps = max_speed_;
    out.received_samples = received_; out.rejected_samples = rejected_;
    out.nonmonotonic_samples = nonmonotonic_; out.gap_events = gaps_;
    out.jump_events = jumps_; out.session_resets = resets_;
    stats_pub_->publish(out);
    last_position_ = p; last_rotation_ = rotation; last_stamp_ = stamp;
    frame_ = input.header.frame_id; child_ = input.child_frame_id; have_last_ = true;
  }
  double expected_rate_, max_gap_, max_step_, speed_limit_;
  double path_length_ = 0, valid_duration_ = 0, max_speed_ = 0;
  bool have_last_ = false;
  int64_t last_stamp_ = 0, first_stamp_ = 0;
  uint64_t received_ = 0, rejected_ = 0, nonmonotonic_ = 0, gaps_ = 0, jumps_ = 0, resets_ = 0;
  geometry_msgs::msg::Point first_position_, last_position_;
  tf2::Quaternion last_rotation_;
  std::string frame_, child_;
  std::deque<int64_t> timestamps_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<sensor_analysis::msg::OdomStats>::SharedPtr stats_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomAnalysis>());
  rclcpp::shutdown();
  return 0;
}
