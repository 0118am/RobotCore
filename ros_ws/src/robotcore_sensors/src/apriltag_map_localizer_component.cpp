#include "robotcore_sensors/apriltag_map.hpp"

#include <robotcore_interfaces/msg/april_tag_pose_estimate.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace robotcore_sensors
{
namespace
{
void set_message_pose(geometry_msgs::msg::Pose & pose, const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond q(transform.linear());
  pose.position.x = transform.translation().x(); pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z(); pose.orientation.x = q.x(); pose.orientation.y = q.y();
  pose.orientation.z = q.z(); pose.orientation.w = q.w();
}
}  // namespace

class AprilTagMapLocalizerComponent final : public rclcpp::Node
{
public:
  explicit AprilTagMapLocalizerComponent(const rclcpp::NodeOptions & options)
  : Node("apriltag_localization", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    camera_topic_ = declare_parameter<std::string>("camera_info_topic", "/zedx/zed_node/rgb/color/rect/camera_info");
    detections_topic_ = declare_parameter<std::string>("detections_topic", "/localization/apriltag/detections");
    map_file_ = declare_parameter<std::string>(
      "tag_map_file", "/etc/robotcore/apriltag_map.json");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    family_ = declare_parameter<std::string>("tag_family", "tag36h11");
    pool_geometry_.enforce = declare_parameter<bool>("enforce_cuboid_pool_geometry", true);
    pool_geometry_.length_m = declare_parameter<double>("pool_length_m", 5.42);
    pool_geometry_.width_m = declare_parameter<double>("pool_width_m", 3.73);
    pool_geometry_.surface_tolerance_m = declare_parameter<double>("pool_surface_tolerance_m", 0.02);
    pool_geometry_.orientation_tolerance_rad =
      declare_parameter<double>("pool_orientation_tolerance_deg", 2.0) * M_PI / 180.0;
    min_tags_ = declare_parameter<int>("minimum_pose_tag_count", 3);
    min_inlier_corners_ = declare_parameter<int>("minimum_inlier_corners_per_tag", 3);
    min_edge_px_ = declare_parameter<double>("min_tag_edge_px", 20.0);
    max_rms_px_ = declare_parameter<double>("max_reprojection_rms_px", 3.0);
    max_reprojection_px_ = declare_parameter<double>("max_reprojection_error_px", 4.0);
    position_stddev_ = declare_parameter<double>("multi_tag_position_stddev_m", 0.05);
    angle_stddev_ = declare_parameter<double>("multi_tag_angle_stddev_deg", 2.0) * M_PI / 180.0;
    load_map();

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    estimate_pub_ = create_publisher<robotcore_interfaces::msg::AprilTagPoseEstimate>(
      declare_parameter<std::string>("pose_topic", "/localization/apriltag_pose"), qos);
    count_pub_ = create_publisher<std_msgs::msg::Int32>(
      declare_parameter<std::string>("detected_count_topic", "/localization/apriltag/detected_count"), qos);
    camera_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(camera_topic_, qos,
      std::bind(&AprilTagMapLocalizerComponent::on_camera, this, std::placeholders::_1));
    detections_sub_ = create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      detections_topic_, qos, std::bind(&AprilTagMapLocalizerComponent::on_detections, this, std::placeholders::_1));
    reload_service_ = create_service<std_srvs::srv::Trigger>(
      declare_parameter<std::string>("relocalize_service", "/localization/apriltag/relocalize"),
      std::bind(&AprilTagMapLocalizerComponent::reload, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  bool load_map()
  {
    try {
      std::ifstream stream(map_file_);
      if (!stream) {throw std::runtime_error("cannot open tag map");}
      nlohmann::json root; stream >> root;
      tags_ = parse_apriltag_map(root, map_frame_, pool_geometry_);
      if (tags_.empty()) {
        map_error_ = "tag map contains no surveyed Tags; localization disabled";
        RCLCPP_WARN(get_logger(), "%s", map_error_.c_str());
      } else {
        map_error_.clear();
        RCLCPP_INFO(get_logger(), "Loaded %zu surveyed AprilTags", tags_.size());
      }
      return true;
    } catch (const std::exception & error) {
      map_error_ = error.what();
      RCLCPP_ERROR(get_logger(), "Tag map rejected: %s", error.what()); return false;
    }
  }

  void reload(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto previous = tags_;
    if (!load_map()) {tags_ = previous; response->success = false; response->message = map_error_; return;}
    ++map_generation_;
    robotcore_interfaces::msg::AprilTagPoseEstimate estimate;
    estimate.header.stamp = now();
    estimate.header.frame_id = map_frame_;
    estimate.map_generation = map_generation_;
    estimate.relocalization_requested = true;
    estimate.pose_valid = false;
    estimate.reprojection_rms_px = std::numeric_limits<float>::quiet_NaN();
    estimate.minimum_tag_edge_px = std::numeric_limits<float>::quiet_NaN();
    estimate.rejection_reason = "tag map reloaded; waiting for a quality-gated mapped pose";
    estimate_pub_->publish(estimate);
    response->success = true;
    response->message = tags_.empty() ?
      "empty tag map loaded; absolute Tag localization disabled" :
      "tag map reloaded; waiting for four quality-gated poses to establish absolute alignment";
  }

  void on_camera(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = message->p[0]; camera_matrix_.at<double>(0, 2) = message->p[2];
    camera_matrix_.at<double>(1, 1) = message->p[5]; camera_matrix_.at<double>(1, 2) = message->p[6];
    camera_matrix_.at<double>(2, 2) = 1.0;
    distortion_ = cv::Mat::zeros(5, 1, CV_64F);
    camera_frame_ = message->header.frame_id;
    camera_ready_ = message->width > 0U && message->height > 0U && !camera_frame_.empty() &&
      std::isfinite(camera_matrix_.at<double>(0, 0)) && std::isfinite(camera_matrix_.at<double>(1, 1)) &&
      std::isfinite(camera_matrix_.at<double>(0, 2)) && std::isfinite(camera_matrix_.at<double>(1, 2)) &&
      camera_matrix_.at<double>(0, 0) > 0.0 && camera_matrix_.at<double>(1, 1) > 0.0;
    if (!camera_ready_) {camera_extrinsic_ready_ = false; return;}
    try {
      base_from_camera_ = tf2::transformToEigen(
        tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero));
      camera_extrinsic_ready_ = true;
    } catch (const std::exception & error) {
      camera_extrinsic_ready_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for camera extrinsic %s <- %s: %s",
        base_frame_.c_str(), camera_frame_.c_str(), error.what());
    }
  }

  void on_detections(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr message)
  {
    robotcore_interfaces::msg::AprilTagPoseEstimate estimate;
    estimate.header = message->header;
    estimate.header.frame_id = map_frame_;
    estimate.map_generation = map_generation_;
    estimate.reprojection_rms_px = std::numeric_limits<float>::quiet_NaN();
    estimate.minimum_tag_edge_px = std::numeric_limits<float>::quiet_NaN();
    std::map<int, const isaac_ros_apriltag_interfaces::msg::AprilTagDetection *> best;
    std::map<int, double> best_area;
    for (const auto & detection : message->detections) {
      if (detection.family != family_) {continue;}
      ++estimate.detected_tag_count;
      double area = 0.0;
      for (std::size_t i = 0; i < 4U; ++i) {
        const auto & a = detection.corners[i]; const auto & b = detection.corners[(i + 1U) % 4U];
        area += a.x * b.y - a.y * b.x;
      }
      area = std::abs(area) * 0.5;
      if (tags_.count(detection.id) && (!best.count(detection.id) || area > best_area[detection.id])) {
        best[detection.id] = &detection; best_area[detection.id] = area;
      }
    }
    std_msgs::msg::Int32 count; count.data = estimate.detected_tag_count; count_pub_->publish(count);
    if (tags_.empty()) {
      estimate.rejection_reason = map_error_.empty() ?
        "tag map contains no surveyed Tags; localization disabled" : map_error_;
      estimate_pub_->publish(estimate);
      return;
    }
    if (!camera_ready_) {estimate.rejection_reason = "waiting for calibrated CameraInfo"; estimate_pub_->publish(estimate); return;}
    if (!camera_extrinsic_ready_) {estimate.rejection_reason = "waiting for camera extrinsic TF"; estimate_pub_->publish(estimate); return;}
    if (message->header.frame_id != camera_frame_) {
      estimate.rejection_reason = "detection/CameraInfo frame mismatch"; estimate_pub_->publish(estimate); return;
    }

    std::vector<cv::Point3d> object_points; std::vector<cv::Point2d> image_points; std::vector<int> seen_ids;
    double minimum_edge = std::numeric_limits<double>::infinity();
    for (const auto & entry : best) {
      const auto & detection = *entry.second;
      double tag_min_edge = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < 4U; ++i) {
        const auto & a = detection.corners[i]; const auto & b = detection.corners[(i + 1U) % 4U];
        tag_min_edge = std::min(tag_min_edge, std::hypot(a.x - b.x, a.y - b.y));
      }
      if (tag_min_edge < min_edge_px_) {continue;}
      minimum_edge = std::min(minimum_edge, tag_min_edge); seen_ids.push_back(entry.first);
      for (std::size_t i = 0; i < 4U; ++i) {
        const auto & object = tags_.at(entry.first).corners[i]; const auto & image = detection.corners[i];
        object_points.emplace_back(object.x(), object.y(), object.z()); image_points.emplace_back(image.x, image.y);
      }
    }
    estimate.mapped_tag_count = static_cast<int>(seen_ids.size());
    if (std::isfinite(minimum_edge)) {estimate.minimum_tag_edge_px = minimum_edge;}
    if (seen_ids.size() < static_cast<std::size_t>(min_tags_)) {
      estimate.rejection_reason = "insufficient mapped Tags"; estimate_pub_->publish(estimate); return;
    }
    cv::Mat rvec, tvec, inliers;
    const bool solved = cv::solvePnPRansac(object_points, image_points, camera_matrix_, distortion_,
      rvec, tvec, false, 100, max_reprojection_px_, 0.999, inliers, cv::SOLVEPNP_ITERATIVE);
    if (!solved || inliers.rows < 4) {estimate.rejection_reason = "joint PnP/RANSAC failed"; estimate_pub_->publish(estimate); return;}
    std::vector<int> counts(seen_ids.size(), 0);
    for (int row = 0; row < inliers.rows; ++row) {
      const int index = inliers.at<int>(row, 0); if (index >= 0 && index < static_cast<int>(counts.size() * 4U)) {++counts[index / 4];}
    }
    for (std::size_t i = 0; i < counts.size(); ++i) {if (counts[i] >= min_inlier_corners_) {++estimate.inlier_tag_count;}}
    if (estimate.inlier_tag_count < min_tags_) {
      estimate.rejection_reason = "insufficient independently supported inlier Tags"; estimate_pub_->publish(estimate); return;
    }
    std::vector<cv::Point2d> projected; cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distortion_, projected);
    double squared = 0.0;
    for (int row = 0; row < inliers.rows; ++row) {
      const int index = inliers.at<int>(row, 0); const double dx = image_points[index].x - projected[index].x;
      const double dy = image_points[index].y - projected[index].y; squared += dx * dx + dy * dy;
    }
    const double rms = std::sqrt(squared / inliers.rows); estimate.reprojection_rms_px = rms;
    if (!std::isfinite(rms) || rms > max_rms_px_) {estimate.rejection_reason = "reprojection RMS gate"; estimate_pub_->publish(estimate); return;}
    cv::Mat rotation_cv; cv::Rodrigues(rvec, rotation_cv);
    Eigen::Matrix3d rotation; Eigen::Vector3d translation;
    for (int row = 0; row < 3; ++row) {
      translation[row] = tvec.at<double>(row, 0);
      for (int col = 0; col < 3; ++col) {rotation(row, col) = rotation_cv.at<double>(row, col);}
    }
    Eigen::Isometry3d camera_from_map = Eigen::Isometry3d::Identity();
    camera_from_map.linear() = rotation; camera_from_map.translation() = translation;
    const Eigen::Isometry3d map_from_base = camera_from_map.inverse() * base_from_camera_.inverse();
    set_message_pose(estimate.pose.pose, map_from_base);
    const double position_var = position_stddev_ * position_stddev_;
    const double angle_var = angle_stddev_ * angle_stddev_;
    estimate.pose.covariance[0] = estimate.pose.covariance[7] = estimate.pose.covariance[14] = position_var;
    estimate.pose.covariance[21] = estimate.pose.covariance[28] = estimate.pose.covariance[35] = angle_var;
    estimate.pose_valid = true;
    estimate_pub_->publish(estimate);
  }

  std::string camera_topic_, detections_topic_, map_file_, map_frame_, base_frame_, family_, map_error_, camera_frame_;
  int min_tags_{}, min_inlier_corners_{}; double min_edge_px_{}, max_rms_px_{}, max_reprojection_px_{};
  double position_stddev_{}, angle_stddev_{};
  std::map<int, TagDefinition> tags_; cv::Mat camera_matrix_, distortion_;
  CuboidPoolGeometry pool_geometry_;
  Eigen::Isometry3d base_from_camera_{Eigen::Isometry3d::Identity()};
  bool camera_ready_{false}, camera_extrinsic_ready_{false};
  std::uint64_t map_generation_{1U};
  tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<robotcore_interfaces::msg::AprilTagPoseEstimate>::SharedPtr estimate_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_sub_;
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
};
}  // namespace robotcore_sensors
RCLCPP_COMPONENTS_REGISTER_NODE(robotcore_sensors::AprilTagMapLocalizerComponent)
