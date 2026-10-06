#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

class OdomAnalysis : public rclcpp::Node
{
public:
  OdomAnalysis() : Node("odom_analysis")
  {
    // 输入话题可通过 ROS 参数修改。
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/Odometry");

    speed_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/analysis/speed", 10);

    vx_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/analysis/vx", 10);

    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic,
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        process(msg);
      });
  }

private:
  void process(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 第一条消息只保存下来，等待下一条才能计算变化。
    if (!previous_) {
      previous_ = msg;
      return;
    }

    const double dt =
      (rclcpp::Time(msg->header.stamp) -
       rclcpp::Time(previous_->header.stamp)).seconds();

    // 时间倒退、重复时间戳或坐标系改变时，重新开始。
    if (dt <= 0.0 ||
        msg->header.frame_id != previous_->header.frame_id) {
      previous_ = msg;
      return;
    }

    const auto & current = msg->pose.pose.position;
    const auto & previous = previous_->pose.pose.position;

    const double vx = (current.x - previous.x) / dt;
    const double vy = (current.y - previous.y) / dt;
    const double vz = (current.z - previous.z) / dt;

    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = std::sqrt(vx * vx + vy * vy + vz * vz);
    speed_pub_->publish(speed_msg);

    std_msgs::msg::Float64 vx_msg;
    vx_msg.data = vx;
    vx_pub_->publish(vx_msg);

    previous_ = msg;
  }

  nav_msgs::msg::Odometry::SharedPtr previous_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vx_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomAnalysis>());
  rclcpp::shutdown();
  return 0;
}
