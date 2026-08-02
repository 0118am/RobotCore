#include "eup_sensors/geometry.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace eup_sensors
{
class ZedOdometryAdapterComponent final : public rclcpp::Node
{
public:
  explicit ZedOdometryAdapterComponent(const rclcpp::NodeOptions & options)
  : Node("zed_odometry_adapter", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/zedx/zed_node/odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/localization/zed_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    min_speed_ = declare_parameter<double>("minimum_linear_speed_mps", 0.002);
    position_floor_ = std::pow(declare_parameter<double>("position_stddev_floor_m", 0.01), 2);
    orientation_floor_ = std::pow(declare_parameter<double>("orientation_stddev_floor_rad", 0.004363323), 2);
    velocity_floor_ = std::pow(declare_parameter<double>("linear_velocity_stddev_floor_mps", 0.03), 2);
    angular_floor_ = std::pow(declare_parameter<double>("angular_velocity_stddev_floor_rps", 0.034906585), 2);
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    output_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, qos);
    input_sub_ = create_subscription<nav_msgs::msg::Odometry>(input_topic_, qos,
      std::bind(&ZedOdometryAdapterComponent::on_odometry, this, std::placeholders::_1));
  }

private:
  static Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose & pose)
  {
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-9) {throw std::runtime_error("invalid pose quaternion");}
    return pose_transform({pose.position.x, pose.position.y, pose.position.z}, q.normalized());
  }

  static void set_pose(geometry_msgs::msg::Pose & pose, const Eigen::Isometry3d & transform)
  {
    const Eigen::Quaterniond q(transform.linear());
    pose.position.x = transform.translation().x(); pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z(); pose.orientation.x = q.x();
    pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
  }

  static Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Isometry3d & base_from_source)
  {
    Eigen::Matrix<double, 6, 6> result = Eigen::Matrix<double, 6, 6>::Zero();
    result.block<3, 3>(0, 0) = base_from_source.linear();
    result.block<3, 3>(0, 3) = skew(base_from_source.translation()) * base_from_source.linear();
    result.block<3, 3>(3, 3) = base_from_source.linear();
    return result;
  }

  template<typename Array>
  static Eigen::Matrix<double, 6, 6> covariance(const Array & values)
  {
    Eigen::Matrix<double, 6, 6> result;
    for (int row = 0; row < 6; ++row) for (int col = 0; col < 6; ++col) {
      const double value = values[6 * row + col];
      result(row, col) = std::isfinite(value) ? value : 0.0;
    }
    return 0.5 * (result + result.transpose());
  }

  template<typename Array>
  static void set_covariance(Array & output, Eigen::Matrix<double, 6, 6> matrix,
    const std::array<double, 6> & floors)
  {
    matrix = 0.5 * (matrix + matrix.transpose());
    for (int index = 0; index < 6; ++index) {matrix(index, index) = std::max(matrix(index, index), floors[index]);}
    for (int row = 0; row < 6; ++row) for (int col = 0; col < 6; ++col) {output[6 * row + col] = matrix(row, col);}
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->child_frame_id.empty()) {return;}
    try {
      const auto transform = tf_buffer_.lookupTransform(base_frame_, message->child_frame_id, tf2::TimePointZero);
      const Eigen::Isometry3d base_from_source = tf2::transformToEigen(transform);
      const Eigen::Isometry3d odom_from_base = pose_to_eigen(message->pose.pose) * base_from_source.inverse();
      nav_msgs::msg::Odometry output;
      output.header = message->header; output.child_frame_id = base_frame_;
      set_pose(output.pose.pose, odom_from_base);
      Eigen::Matrix<double, 6, 1> source_twist;
      source_twist << message->twist.twist.linear.x, message->twist.twist.linear.y,
        message->twist.twist.linear.z, message->twist.twist.angular.x,
        message->twist.twist.angular.y, message->twist.twist.angular.z;
      Eigen::Matrix<double, 6, 1> base_twist = adjoint(base_from_source) * source_twist;
      if (base_twist.head<3>().norm() < min_speed_) {base_twist.head<3>().setZero();}
      output.twist.twist.linear.x = base_twist[0]; output.twist.twist.linear.y = base_twist[1];
      output.twist.twist.linear.z = base_twist[2]; output.twist.twist.angular.x = base_twist[3];
      output.twist.twist.angular.y = base_twist[4]; output.twist.twist.angular.z = base_twist[5];
      const auto a = adjoint(base_from_source);
      set_covariance(output.pose.covariance, a * covariance(message->pose.covariance) * a.transpose(),
        {position_floor_, position_floor_, position_floor_, orientation_floor_, orientation_floor_, orientation_floor_});
      set_covariance(output.twist.covariance, a * covariance(message->twist.covariance) * a.transpose(),
        {velocity_floor_, velocity_floor_, velocity_floor_, angular_floor_, angular_floor_, angular_floor_});
      output_pub_->publish(output);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for VIO transform: %s", error.what());
    }
  }

  std::string input_topic_, output_topic_, base_frame_;
  double min_speed_{}, position_floor_{}, orientation_floor_{}, velocity_floor_{}, angular_floor_{};
  tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_sub_;
};
}  // namespace eup_sensors
RCLCPP_COMPONENTS_REGISTER_NODE(eup_sensors::ZedOdometryAdapterComponent)
