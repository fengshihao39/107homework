#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

class ImuAnalysis : public rclcpp::Node
{
public:
  ImuAnalysis() : Node("imu_analysis")
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/livox/imu");

    // 当前 Livox 驱动的输入单位是 g，转换为 m/s²。
    // 如果换用已经输出 m/s² 的驱动，应设置为 1.0。
    accel_scale_ = declare_parameter<double>("accel_scale", 9.80665);

    accel_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/analysis/accel_norm", 10);

    gyro_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/analysis/gyro_norm", 10);

    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        process(msg);
      });
  }

private:
  void process(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & a = msg->linear_acceleration;
    const auto & w = msg->angular_velocity;

    std_msgs::msg::Float64 accel_msg;
    accel_msg.data =
      std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z) * accel_scale_;
    accel_pub_->publish(accel_msg);

    std_msgs::msg::Float64 gyro_msg;
    gyro_msg.data =
      std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
    gyro_pub_->publish(gyro_msg);
  }

  double accel_scale_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gyro_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuAnalysis>());
  rclcpp::shutdown();
  return 0;
}
