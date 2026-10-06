#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_analysis/msg/imu_stats.hpp"

class ImuAnalysis : public rclcpp::Node {
 public:
  ImuAnalysis() : Node("imu_analysis") {
    const auto topic = declare_parameter<std::string>("input_topic", "/livox/imu");
    scale_ = positive("acceleration_scale", 9.80665);
    gravity_ = positive("gravity_mps2", 9.80665);
    window_seconds_ = positive("window_seconds", 1.0);
    publish_period_ = positive("publish_period_seconds", 0.1);
    expected_rate_ = positive("expected_rate_hz", 200.0);
    gyro_threshold_ = positive("stationary_gyro_rms_radps", 0.05);
    std_threshold_ = positive("stationary_accel_std_mps2", 0.15);
    gravity_threshold_ = positive("stationary_gravity_error_mps2", 0.30);
    stats_pub_ = create_publisher<sensor_analysis::msg::ImuStats>("/analysis/imu_stats", 10);
    si_pub_ = create_publisher<sensor_msgs::msg::Imu>("/analysis/imu_si", 100);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS().keep_last(1000),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(*msg); });
    RCLCPP_INFO(get_logger(), "IMU input %s, acceleration scale %.5f; output SI units", topic.c_str(), scale_);
  }

 private:
  struct Sample { int64_t stamp; double accel; double gyro2; };
  double positive(const std::string &name, double initial) {
    const auto value = declare_parameter<double>(name, initial);
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(name + " must be positive and finite");
    return value;
  }
  void reset() {
    samples_.clear(); sum_accel_ = sum_accel2_ = sum_gyro2_ = 0.0;
    have_last_ = false; have_publish_ = false;
    ++resets_;
  }
  void on_imu(const sensor_msgs::msg::Imu &input) {
    ++received_;
    const auto &a = input.linear_acceleration;
    const auto &g = input.angular_velocity;
    const auto stamp = rclcpp::Time(input.header.stamp).nanoseconds();
    const double accel = std::hypot(a.x, a.y, a.z) * scale_;
    const double gyro = std::hypot(g.x, g.y, g.z);
    if (!std::isfinite(accel) || !std::isfinite(gyro) ||
        !std::isfinite(accel * accel) || !std::isfinite(gyro * gyro)) {
      ++rejected_; return;
    }
    if (have_last_ && input.header.frame_id != frame_) reset();
    if (have_last_ && stamp <= last_stamp_) {
      ++nonmonotonic_;
      if (stamp == last_stamp_) return;
      // A bag loop/time rewind starts a new segment; never mix the windows.
      reset();
    }
    if (have_last_ && (stamp - last_stamp_) / 1e9 > 2.5 / expected_rate_) {
      ++gaps_;
      // An interrupted stream needs a fresh window before stationary detection.
      samples_.clear(); sum_accel_ = sum_accel2_ = sum_gyro2_ = 0.0;
    }
    last_stamp_ = stamp; have_last_ = true; frame_ = input.header.frame_id;
    auto si = input;
    si.linear_acceleration.x *= scale_;
    si.linear_acceleration.y *= scale_;
    si.linear_acceleration.z *= scale_;
    for (auto &value : si.linear_acceleration_covariance) value *= scale_ * scale_;
    // Livox supplies gyro/acceleration, not a fused orientation measurement.
    si.orientation_covariance[0] = -1.0;
    si_pub_->publish(si);
    samples_.push_back({stamp, accel, gyro * gyro});
    sum_accel_ += accel; sum_accel2_ += accel * accel; sum_gyro2_ += gyro * gyro;
    while (samples_.size() > 1 && (stamp - samples_.front().stamp) / 1e9 > window_seconds_) {
      const auto old = samples_.front(); samples_.pop_front();
      sum_accel_ -= old.accel; sum_accel2_ -= old.accel * old.accel; sum_gyro2_ -= old.gyro2;
    }
    if (have_publish_ && (stamp - last_publish_) / 1e9 < publish_period_) return;
    sensor_analysis::msg::ImuStats out;
    out.header = input.header;
    out.window_samples = static_cast<uint32_t>(samples_.size());
    const double count = static_cast<double>(samples_.size());
    out.window_seconds = (stamp - samples_.front().stamp) / 1e9;
    out.sample_rate_hz = out.window_seconds > 0 ? (count - 1.0) / out.window_seconds : 0.0;
    out.accel_norm_mean_mps2 = sum_accel_ / count;
    out.accel_norm_std_mps2 = std::sqrt(std::max(0.0, sum_accel2_ / count - std::pow(out.accel_norm_mean_mps2, 2)));
    out.gyro_norm_rms_radps = std::sqrt(std::max(0.0, sum_gyro2_ / count));
    out.gravity_norm_deviation_mps2 = std::abs(out.accel_norm_mean_mps2 - gravity_);
    out.stationary_candidate = out.window_seconds >= window_seconds_ * 0.5 &&
      out.gyro_norm_rms_radps < gyro_threshold_ && out.accel_norm_std_mps2 < std_threshold_ &&
      out.gravity_norm_deviation_mps2 < gravity_threshold_;
    out.received_samples = received_; out.rejected_samples = rejected_;
    out.nonmonotonic_samples = nonmonotonic_; out.gap_events = gaps_; out.session_resets = resets_;
    stats_pub_->publish(out);
    last_publish_ = stamp; have_publish_ = true;
  }
  double scale_, gravity_, window_seconds_, publish_period_, expected_rate_;
  double gyro_threshold_, std_threshold_, gravity_threshold_;
  double sum_accel_ = 0, sum_accel2_ = 0, sum_gyro2_ = 0;
  int64_t last_stamp_ = 0, last_publish_ = 0;
  bool have_last_ = false, have_publish_ = false;
  uint64_t received_ = 0, rejected_ = 0, nonmonotonic_ = 0, gaps_ = 0, resets_ = 0;
  std::string frame_;
  std::deque<Sample> samples_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr si_pub_;
  rclcpp::Publisher<sensor_analysis::msg::ImuStats>::SharedPtr stats_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuAnalysis>());
  rclcpp::shutdown();
  return 0;
}
