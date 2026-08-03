#include "robotcore_sensors/apriltag_map.hpp"
#include "robotcore_sensors/geometry.hpp"

#include <robotcore_interfaces/msg/april_tag_pose_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace robotcore_sensors
{
namespace
{
Eigen::Isometry3d message_pose(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-9 || !q.coeffs().allFinite()) {throw std::runtime_error("invalid quaternion");}
  return pose_transform({pose.position.x, pose.position.y, pose.position.z}, q.normalized());
}

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
  : Node("apriltag_localization", options)
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
    max_jump_m_ = declare_parameter<double>("max_translation_jump_m", 0.05);
    max_speed_mps_ = declare_parameter<double>("max_translation_speed_mps", 1.0);
    const auto translation = declare_parameter<std::vector<double>>(
      "base_to_camera_translation_m", {0.236, 0.027, 0.016});
    const auto rpy = declare_parameter<std::vector<double>>(
      "base_to_camera_optical_rpy_rad", {-M_PI / 2.0, 0.0, -M_PI / 2.0});
    if (translation.size() != 3U || rpy.size() != 3U) {throw std::runtime_error("camera extrinsic requires 3-vectors");}
    base_from_camera_ = Eigen::Isometry3d::Identity();
    base_from_camera_.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    base_from_camera_.linear() = fixed_axis_rpy_rotation({rpy[0], rpy[1], rpy[2]});
    load_map();

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter<std::string>("pose_topic", "/localization/apriltag_pose"), qos);
    degraded_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter<std::string>("degraded_pose_topic", "/localization/apriltag_pose_degraded"), qos);
    count_pub_ = create_publisher<std_msgs::msg::Int32>(
      declare_parameter<std::string>("detected_count_topic", "/localization/apriltag/detected_count"), qos);
    status_pub_ = create_publisher<robotcore_interfaces::msg::AprilTagPoseStatus>(
      declare_parameter<std::string>("pose_status_topic", "/localization/apriltag/pose_status"), qos);
    camera_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(camera_topic_, qos,
      std::bind(&AprilTagMapLocalizerComponent::on_camera, this, std::placeholders::_1));
    detections_sub_ = create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      detections_topic_, qos, std::bind(&AprilTagMapLocalizerComponent::on_detections, this, std::placeholders::_1));
    aligned_vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("aligned_vio_odometry_topic", "/localization/aligned_vio_odom"), qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {last_aligned_vio_ = msg;});
    reload_service_ = create_service<std_srvs::srv::Trigger>(
      declare_parameter<std::string>("relocalize_service", "/localization/apriltag/relocalize"),
      std::bind(&AprilTagMapLocalizerComponent::reload, this, std::placeholders::_1, std::placeholders::_2));
    estimator_relocalize_client_ = create_client<std_srvs::srv::Trigger>(
      "/localization/tag_vio/relocalize");
    relocalize_event_pub_ = create_publisher<std_msgs::msg::Empty>(
      "/localization/relocalize_event", rclcpp::QoS(1).reliable());
    relocalize_event_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/localization/relocalize_event", rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Empty::SharedPtr) {
        have_last_pose_ = false; relocalization_pending_ = true;
      });
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
    have_last_pose_ = false; relocalization_pending_ = true;
    if (estimator_relocalize_client_->service_is_ready()) {
      estimator_relocalize_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    relocalize_event_pub_->publish(std_msgs::msg::Empty{});
    response->success = true;
    response->message = tags_.empty() ?
      "empty tag map loaded; absolute Tag localization disabled" :
      "tag map reloaded; next quality-gated pose bypasses transition gate";
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
  }

  void on_detections(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr message)
  {
    robotcore_interfaces::msg::AprilTagPoseStatus status; status.header = message->header;
    status.reprojection_rms_px = std::numeric_limits<float>::quiet_NaN();
    status.minimum_tag_edge_px = std::numeric_limits<float>::quiet_NaN();
    std::map<int, const isaac_ros_apriltag_interfaces::msg::AprilTagDetection *> best;
    std::map<int, double> best_area;
    for (const auto & detection : message->detections) {
      if (detection.family != family_) {continue;}
      ++status.detected_tag_count;
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
    std_msgs::msg::Int32 count; count.data = status.detected_tag_count; count_pub_->publish(count);
    if (tags_.empty()) {
      status.rejection_reason = map_error_.empty() ?
        "tag map contains no surveyed Tags; localization disabled" : map_error_;
      status_pub_->publish(status);
      return;
    }
    if (!camera_ready_) {status.rejection_reason = "waiting for calibrated CameraInfo"; status_pub_->publish(status); return;}
    if (message->header.frame_id != camera_frame_) {
      status.rejection_reason = "detection/CameraInfo frame mismatch"; status_pub_->publish(status); return;
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
    status.mapped_tag_count = static_cast<int>(seen_ids.size());
    if (std::isfinite(minimum_edge)) {status.minimum_tag_edge_px = minimum_edge;}
    const bool degraded = seen_ids.size() == 2U;
    if (seen_ids.size() < static_cast<std::size_t>(min_tags_) && !degraded) {
      status.rejection_reason = "insufficient mapped Tags"; status_pub_->publish(status); return;
    }
    cv::Mat rvec, tvec, inliers;
    const bool solved = cv::solvePnPRansac(object_points, image_points, camera_matrix_, distortion_,
      rvec, tvec, false, 100, max_reprojection_px_, 0.999, inliers, cv::SOLVEPNP_ITERATIVE);
    if (!solved || inliers.rows < 4) {status.rejection_reason = "joint PnP/RANSAC failed"; status_pub_->publish(status); return;}
    std::vector<int> counts(seen_ids.size(), 0);
    for (int row = 0; row < inliers.rows; ++row) {
      const int index = inliers.at<int>(row, 0); if (index >= 0 && index < static_cast<int>(counts.size() * 4U)) {++counts[index / 4];}
    }
    const int required_corners = degraded ? 4 : min_inlier_corners_;
    for (std::size_t i = 0; i < counts.size(); ++i) {if (counts[i] >= required_corners) {++status.inlier_tag_count;}}
    if (status.inlier_tag_count < (degraded ? 2 : min_tags_)) {
      status.rejection_reason = "insufficient independently supported inlier Tags"; status_pub_->publish(status); return;
    }
    std::vector<cv::Point2d> projected; cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distortion_, projected);
    double squared = 0.0;
    for (int row = 0; row < inliers.rows; ++row) {
      const int index = inliers.at<int>(row, 0); const double dx = image_points[index].x - projected[index].x;
      const double dy = image_points[index].y - projected[index].y; squared += dx * dx + dy * dy;
    }
    const double rms = std::sqrt(squared / inliers.rows); status.reprojection_rms_px = rms;
    if (!std::isfinite(rms) || rms > max_rms_px_) {status.rejection_reason = "reprojection RMS gate"; status_pub_->publish(status); return;}
    cv::Mat rotation_cv; cv::Rodrigues(rvec, rotation_cv);
    Eigen::Matrix3d rotation; Eigen::Vector3d translation;
    for (int row = 0; row < 3; ++row) {
      translation[row] = tvec.at<double>(row, 0);
      for (int col = 0; col < 3; ++col) {rotation(row, col) = rotation_cv.at<double>(row, col);}
    }
    Eigen::Isometry3d camera_from_map = Eigen::Isometry3d::Identity();
    camera_from_map.linear() = rotation; camera_from_map.translation() = translation;
    const Eigen::Isometry3d map_from_base = camera_from_map.inverse() * base_from_camera_.inverse();
    if (degraded && !degraded_consistent(map_from_base, rclcpp::Time(message->header.stamp))) {
      status.rejection_reason = "two-Tag VIO validation gate"; status_pub_->publish(status); return;
    }
    if (!degraded && have_last_pose_ && !relocalization_pending_) {
      const double dt = std::max(0.0, (rclcpp::Time(message->header.stamp) - last_pose_stamp_).seconds());
      if ((map_from_base.translation() - last_pose_.translation()).norm() > max_jump_m_ + max_speed_mps_ * std::min(dt, 0.25)) {
        status.rejection_reason = "pose transition gate"; status_pub_->publish(status); return;
      }
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose; pose.header = message->header; pose.header.frame_id = map_frame_;
    set_message_pose(pose.pose.pose, map_from_base);
    const double position_var = position_stddev_ * position_stddev_ * (degraded ? 4.0 : 1.0);
    const double angle_var = angle_stddev_ * angle_stddev_ * (degraded ? 4.0 : 1.0);
    pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[14] = position_var;
    pose.pose.covariance[21] = pose.pose.covariance[28] = pose.pose.covariance[35] = angle_var;
    (degraded ? degraded_pub_ : pose_pub_)->publish(pose);
    status.pose_published = true; status.degraded = degraded;
    if (!degraded) {last_pose_ = map_from_base; last_pose_stamp_ = rclcpp::Time(message->header.stamp); have_last_pose_ = true; relocalization_pending_ = false;}
    status_pub_->publish(status);
  }

  bool degraded_consistent(const Eigen::Isometry3d & observed, const rclcpp::Time & stamp) const
  {
    if (!last_aligned_vio_) {return false;}
    if (std::abs((stamp - rclcpp::Time(last_aligned_vio_->header.stamp)).seconds()) > 0.05) {return false;}
    const auto predicted = message_pose(last_aligned_vio_->pose.pose);
    return (observed.translation() - predicted.translation()).norm() <= 0.20 &&
      rotation_distance(observed.linear(), predicted.linear()) <= 10.0 * M_PI / 180.0;
  }

  std::string camera_topic_, detections_topic_, map_file_, map_frame_, base_frame_, family_, map_error_, camera_frame_;
  int min_tags_{}, min_inlier_corners_{}; double min_edge_px_{}, max_rms_px_{}, max_reprojection_px_{};
  double position_stddev_{}, angle_stddev_{}, max_jump_m_{}, max_speed_mps_{};
  std::map<int, TagDefinition> tags_; cv::Mat camera_matrix_, distortion_;
  CuboidPoolGeometry pool_geometry_;
  Eigen::Isometry3d base_from_camera_{Eigen::Isometry3d::Identity()}, last_pose_{Eigen::Isometry3d::Identity()};
  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME}; bool camera_ready_{false}, have_last_pose_{false}, relocalization_pending_{false};
  nav_msgs::msg::Odometry::SharedPtr last_aligned_vio_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_, degraded_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_pub_;
  rclcpp::Publisher<robotcore_interfaces::msg::AprilTagPoseStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_sub_;
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr aligned_vio_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr estimator_relocalize_client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr relocalize_event_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr relocalize_event_sub_;
};
}  // namespace robotcore_sensors
RCLCPP_COMPONENTS_REGISTER_NODE(robotcore_sensors::AprilTagMapLocalizerComponent)
