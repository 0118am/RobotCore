#include "robotcore_sensors/apriltag_map.hpp"

#include <robotcore_interfaces/msg/april_tag_pose_estimate.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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
    min_tags_ = std::max(
      1, static_cast<int>(declare_parameter<int>("minimum_pose_tag_count", 1)));
    selected_tag_count_ = std::max(
      min_tags_, static_cast<int>(declare_parameter<int>("maximum_pose_tag_count", 3)));
    min_inlier_corners_ = declare_parameter<int>("minimum_inlier_corners_per_tag", 3);
    min_edge_px_ = declare_parameter<double>("min_tag_edge_px", 20.0);
    max_edge_ratio_ = declare_parameter<double>("maximum_tag_edge_ratio", 2.5);
    max_rms_px_ = declare_parameter<double>("max_reprojection_rms_px", 3.0);
    single_tag_max_rms_px_ = declare_parameter<double>(
      "single_tag_max_reprojection_rms_px", 1.5);
    max_reprojection_px_ = declare_parameter<double>("max_reprojection_error_px", 4.0);
    position_stddev_ = declare_parameter<double>("multi_tag_position_stddev_m", 0.05);
    angle_stddev_ = declare_parameter<double>("multi_tag_angle_stddev_deg", 2.0) * M_PI / 180.0;
    dual_tag_position_stddev_ = declare_parameter<double>("dual_tag_position_stddev_m", 0.075);
    dual_tag_angle_stddev_ =
      declare_parameter<double>("dual_tag_angle_stddev_deg", 3.0) * M_PI / 180.0;
    single_tag_position_stddev_ = declare_parameter<double>("single_tag_position_stddev_m", 0.10);
    single_tag_angle_stddev_ = declare_parameter<double>(
      "single_tag_angle_stddev_deg", 4.0) * M_PI / 180.0;
    single_tag_reference_edge_px_ = declare_parameter<double>("single_tag_reference_edge_px", 40.0);
    load_map();

    const auto camera_qos = rclcpp::SensorDataQoS().keep_last(1);
    // Tag corrections are sparse absolute measurements.  Request delivery
    // from the reliable Isaac ROS publisher, but retain only the newest
    // estimate so an old correction cannot build up behind the estimator.
    const auto tag_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    estimate_pub_ = create_publisher<robotcore_interfaces::msg::AprilTagPoseEstimate>(
      declare_parameter<std::string>("pose_topic", "/localization/apriltag_pose"), tag_qos);
    camera_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(camera_topic_, camera_qos,
      std::bind(&AprilTagMapLocalizerComponent::on_camera, this, std::placeholders::_1));
    detections_sub_ = create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      detections_topic_, tag_qos,
      std::bind(&AprilTagMapLocalizerComponent::on_detections, this, std::placeholders::_1));
    reload_service_ = create_service<std_srvs::srv::Trigger>(
      declare_parameter<std::string>("relocalize_service", "/localization/apriltag/relocalize"),
      std::bind(&AprilTagMapLocalizerComponent::reload, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void load_map()
  {
    std::ifstream stream(map_file_);
    nlohmann::json root;
    stream >> root;
    tags_ = parse_apriltag_map(root, map_frame_, pool_geometry_);
    RCLCPP_INFO(get_logger(), "Loaded %zu surveyed AprilTags", tags_.size());
  }

  void reload(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    load_map();
    ++map_generation_;
    robotcore_interfaces::msg::AprilTagPoseEstimate estimate;
    estimate.header.stamp = now();
    estimate.header.frame_id = map_frame_;
    estimate.map_generation = map_generation_;
    estimate.relocalization_requested = true;
    estimate.pose_valid = false;
    estimate.reprojection_rms_px = std::numeric_limits<float>::quiet_NaN();
    estimate.minimum_tag_edge_px = std::numeric_limits<float>::quiet_NaN();
    estimate.rejection_reason = "tag map reloaded; estimator alignment reset";
    estimate_pub_->publish(estimate);
    response->success = true;
    response->message = "tag map reloaded; estimator alignment reset";
  }

  void on_camera(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    if (camera_ready_ && camera_extrinsic_ready_ &&
      message->header.frame_id == camera_frame_ &&
      message->width == camera_width_ && message->height == camera_height_ &&
      message->p[0] == camera_matrix_(0, 0) && message->p[2] == camera_matrix_(0, 2) &&
      message->p[5] == camera_matrix_(1, 1) && message->p[6] == camera_matrix_(1, 2))
    {
      return;
    }
    camera_matrix_ = cv::Matx33d::zeros();
    camera_matrix_(0, 0) = message->p[0]; camera_matrix_(0, 2) = message->p[2];
    camera_matrix_(1, 1) = message->p[5]; camera_matrix_(1, 2) = message->p[6];
    camera_matrix_(2, 2) = 1.0;
    camera_width_ = message->width; camera_height_ = message->height;
    camera_frame_ = message->header.frame_id;
    camera_ready_ = message->width > 0U && message->height > 0U && !camera_frame_.empty() &&
      std::isfinite(camera_matrix_(0, 0)) && std::isfinite(camera_matrix_(1, 1)) &&
      std::isfinite(camera_matrix_(0, 2)) && std::isfinite(camera_matrix_(1, 2)) &&
      camera_matrix_(0, 0) > 0.0 && camera_matrix_(1, 1) > 0.0;
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
    if (!camera_ready_) {estimate.rejection_reason = "waiting for calibrated CameraInfo"; estimate_pub_->publish(estimate); return;}
    if (!camera_extrinsic_ready_) {estimate.rejection_reason = "waiting for camera extrinsic TF"; estimate_pub_->publish(estimate); return;}
    if (message->header.frame_id != camera_frame_) {
      estimate.rejection_reason = "detection/CameraInfo frame mismatch"; estimate_pub_->publish(estimate); return;
    }

    struct Candidate
    {
      int id{};
      const isaac_ros_apriltag_interfaces::msg::AprilTagDetection * detection{};
      TagImageQuality quality;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(best.size());
    for (const auto & entry : best) {
      std::array<Eigen::Vector2d, 4> corners;
      for (std::size_t index = 0; index < corners.size(); ++index) {
        corners[index] = {entry.second->corners[index].x, entry.second->corners[index].y};
      }
      const auto quality = assess_tag_image_quality(
        corners, camera_width_, camera_height_, min_edge_px_, max_edge_ratio_);
      if (quality.accepted) {candidates.push_back({entry.first, entry.second, quality});}
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate & left, const Candidate & right) {
      if (left.quality.score != right.quality.score) {
        return left.quality.score > right.quality.score;
      }
      return left.id < right.id;
    });
    if (candidates.size() < static_cast<std::size_t>(min_tags_)) {
      estimate.mapped_tag_count = static_cast<int>(candidates.size());
      estimate.rejection_reason = "no quality-gated mapped Tag available";
      estimate_pub_->publish(estimate);
      return;
    }
    candidates.resize(std::min(candidates.size(), static_cast<std::size_t>(selected_tag_count_)));

    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;
    std::vector<int> seen_ids;
    object_points.reserve(candidates.size() * 4U);
    image_points.reserve(candidates.size() * 4U);
    seen_ids.reserve(candidates.size());
    double minimum_edge = std::numeric_limits<double>::infinity();
    for (const auto & candidate : candidates) {
      const auto & detection = *candidate.detection;
      minimum_edge = std::min(minimum_edge, candidate.quality.minimum_edge_px);
      seen_ids.push_back(candidate.id);
      for (std::size_t i = 0; i < 4U; ++i) {
        const auto & object = tags_.at(candidate.id).corners[i];
        const auto & image = detection.corners[i];
        object_points.emplace_back(object.x(), object.y(), object.z()); image_points.emplace_back(image.x, image.y);
      }
    }
    estimate.mapped_tag_count = static_cast<int>(seen_ids.size());
    if (std::isfinite(minimum_edge)) {estimate.minimum_tag_edge_px = minimum_edge;}
    if (seen_ids.size() < static_cast<std::size_t>(min_tags_)) {
      estimate.rejection_reason = "insufficient mapped Tags"; estimate_pub_->publish(estimate); return;
    }
    const bool single_tag = seen_ids.size() == 1U;
    cv::Mat rvec, tvec, inliers;
    bool solved = false;
    if (single_tag) {
      std::vector<cv::Mat> rotation_solutions;
      std::vector<cv::Mat> translation_solutions;
      solved = cv::solvePnPGeneric(
        object_points, image_points, camera_matrix_, distortion_,
        rotation_solutions, translation_solutions, false, cv::SOLVEPNP_IPPE) > 0;
      if (solved) {
        double best_rms = std::numeric_limits<double>::infinity();
        std::size_t best_solution = rotation_solutions.size();
        for (std::size_t solution = 0;
          solution < rotation_solutions.size() && solution < translation_solutions.size();
          ++solution)
        {
          cv::Mat candidate_rotation;
          cv::Rodrigues(rotation_solutions[solution], candidate_rotation);
          bool all_points_in_front = true;
          for (const auto & point : object_points) {
            const double depth =
              candidate_rotation.at<double>(2, 0) * point.x +
              candidate_rotation.at<double>(2, 1) * point.y +
              candidate_rotation.at<double>(2, 2) * point.z +
              translation_solutions[solution].at<double>(2, 0);
            if (!std::isfinite(depth) || depth <= 0.0) {
              all_points_in_front = false;
              break;
            }
          }
          if (!all_points_in_front) {continue;}

          std::vector<cv::Point2d> candidate_projection;
          cv::projectPoints(
            object_points, rotation_solutions[solution], translation_solutions[solution],
            camera_matrix_, distortion_, candidate_projection);
          double squared_error = 0.0;
          for (std::size_t index = 0; index < image_points.size(); ++index) {
            const double dx = image_points[index].x - candidate_projection[index].x;
            const double dy = image_points[index].y - candidate_projection[index].y;
            squared_error += dx * dx + dy * dy;
          }
          const double candidate_rms = std::sqrt(squared_error / image_points.size());
          if (std::isfinite(candidate_rms) && candidate_rms < best_rms) {
            best_rms = candidate_rms;
            best_solution = solution;
          }
        }
        solved = best_solution < rotation_solutions.size();
        if (solved) {
          rvec = rotation_solutions[best_solution];
          tvec = translation_solutions[best_solution];
          inliers = (cv::Mat_<int>(4, 1) << 0, 1, 2, 3);
        }
      }
    } else {
      solved = cv::solvePnPRansac(
        object_points, image_points, camera_matrix_, distortion_,
        rvec, tvec, false, 100, max_reprojection_px_, 0.999,
        inliers, cv::SOLVEPNP_ITERATIVE);
    }
    if (!solved || inliers.rows < 4) {
      estimate.rejection_reason = single_tag ?
        "single Tag planar PnP failed" : "selected-Tag joint PnP/RANSAC failed";
      estimate_pub_->publish(estimate); return;
    }
    std::vector<int> inlier_point_indices;
    inlier_point_indices.reserve(static_cast<std::size_t>(inliers.rows));
    for (int row = 0; row < inliers.rows; ++row) {
      inlier_point_indices.push_back(inliers.at<int>(row, 0));
    }
    const auto supported_tag_indices = independently_supported_tag_indices(
      inlier_point_indices, seen_ids.size(), 4U, min_inlier_corners_);
    estimate.inlier_tag_count = static_cast<int>(supported_tag_indices.size());
    if (estimate.inlier_tag_count < min_tags_) {
      estimate.rejection_reason = "insufficient independently supported mapped Tags";
      estimate_pub_->publish(estimate); return;
    }

    std::vector<bool> supported_tag_mask(seen_ids.size(), false);
    minimum_edge = std::numeric_limits<double>::infinity();
    for (const auto tag_index : supported_tag_indices) {
      supported_tag_mask[tag_index] = true;
      minimum_edge = std::min(minimum_edge, candidates[tag_index].quality.minimum_edge_px);
    }
    std::vector<cv::Point3d> supported_object_points;
    std::vector<cv::Point2d> supported_image_points;
    supported_object_points.reserve(inlier_point_indices.size());
    supported_image_points.reserve(inlier_point_indices.size());
    for (const int raw_index : inlier_point_indices) {
      if (raw_index < 0) {continue;}
      const auto point_index = static_cast<std::size_t>(raw_index);
      if (point_index >= object_points.size() || !supported_tag_mask[point_index / 4U]) {continue;}
      supported_object_points.push_back(object_points[point_index]);
      supported_image_points.push_back(image_points[point_index]);
    }
    try {
      cv::solvePnPRefineLM(
        supported_object_points, supported_image_points, camera_matrix_, distortion_, rvec, tvec);
    } catch (const cv::Exception &) {
      estimate.rejection_reason = "supported-Tag pose refinement failed";
      estimate_pub_->publish(estimate); return;
    }
    if (std::isfinite(minimum_edge)) {estimate.minimum_tag_edge_px = minimum_edge;}

    std::vector<cv::Point2d> projected;
    cv::projectPoints(
      supported_object_points, rvec, tvec, camera_matrix_, distortion_, projected);
    double squared = 0.0;
    for (std::size_t index = 0; index < projected.size(); ++index) {
      const double dx = supported_image_points[index].x - projected[index].x;
      const double dy = supported_image_points[index].y - projected[index].y;
      squared += dx * dx + dy * dy;
    }
    const double rms = std::sqrt(squared / static_cast<double>(projected.size()));
    estimate.reprojection_rms_px = rms;
    const bool final_single_tag = supported_tag_indices.size() == 1U;
    const double rms_limit = final_single_tag ? single_tag_max_rms_px_ : max_rms_px_;
    if (!std::isfinite(rms) || rms > rms_limit) {
      estimate.rejection_reason = "supported-Tag reprojection RMS gate";
      estimate_pub_->publish(estimate); return;
    }
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
    double position_stddev = position_stddev_;
    double angle_stddev = angle_stddev_;
    if (final_single_tag) {
      const double edge_scale = std::sqrt(std::max(1.0, single_tag_reference_edge_px_ / minimum_edge));
      const double rms_scale = std::max(1.0, rms / 0.75);
      const double quality_scale = std::min(3.0, std::max(edge_scale, rms_scale));
      position_stddev = single_tag_position_stddev_ * quality_scale;
      angle_stddev = single_tag_angle_stddev_ * quality_scale;
    } else if (supported_tag_indices.size() == 2U) {
      position_stddev = dual_tag_position_stddev_;
      angle_stddev = dual_tag_angle_stddev_;
    }
    const double position_var = position_stddev * position_stddev;
    const double angle_var = angle_stddev * angle_stddev;
    estimate.pose.covariance[0] = estimate.pose.covariance[7] = estimate.pose.covariance[14] = position_var;
    estimate.pose.covariance[21] = estimate.pose.covariance[28] = estimate.pose.covariance[35] = angle_var;
    estimate.pose_valid = true;
    estimate_pub_->publish(estimate);
  }

  std::string camera_topic_, detections_topic_, map_file_, map_frame_, base_frame_, family_, camera_frame_;
  int min_tags_{}, selected_tag_count_{}, min_inlier_corners_{};
  double min_edge_px_{}, max_edge_ratio_{}, max_rms_px_{}, single_tag_max_rms_px_{};
  double max_reprojection_px_{}, position_stddev_{}, angle_stddev_{};
  double dual_tag_position_stddev_{}, dual_tag_angle_stddev_{}, single_tag_position_stddev_{};
  double single_tag_angle_stddev_{}, single_tag_reference_edge_px_{};
  std::map<int, TagDefinition> tags_;
  cv::Matx33d camera_matrix_{cv::Matx33d::zeros()};
  cv::Vec<double, 5> distortion_{0.0, 0.0, 0.0, 0.0, 0.0};
  std::uint32_t camera_width_{}, camera_height_{};
  CuboidPoolGeometry pool_geometry_;
  Eigen::Isometry3d base_from_camera_{Eigen::Isometry3d::Identity()};
  bool camera_ready_{false}, camera_extrinsic_ready_{false};
  std::uint64_t map_generation_{1U};
  tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<robotcore_interfaces::msg::AprilTagPoseEstimate>::SharedPtr estimate_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_sub_;
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
};
}  // namespace robotcore_sensors
RCLCPP_COMPONENTS_REGISTER_NODE(robotcore_sensors::AprilTagMapLocalizerComponent)
