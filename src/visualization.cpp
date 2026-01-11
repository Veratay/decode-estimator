#include "decode_estimator/visualization.hpp"

#if DECODE_ENABLE_RERUN

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace decode {

Visualizer::Visualizer(const std::string &app_id) : rec_(app_id) {
  // Default behavior for backward compatibility
  rec_.spawn().exit_on_failure();
}

Visualizer::Visualizer(const VisualizationConfig& config) : rec_(config.app_id) {
    configure(config);
}

Visualizer::~Visualizer() = default;

void Visualizer::setEnabled(bool enabled) {
    enabled_ = enabled;
}

bool Visualizer::isEnabled() const {
    return enabled_;
}

void Visualizer::configure(const VisualizationConfig& config) {
    enabled_ = config.enabled;
    if (!enabled_) return;

    if (config.stream_to_viewer) {
        // connect_grpc is the correct method in newer SDKs
        // ignore error for now or log it if we had a logger
        (void)rec_.connect_grpc(config.stream_url); 
    } else if (!config.save_path.empty()) {
        (void)rec_.save(config.save_path);
    } else {
        // Fallback to spawn if no path/url (mostly for desktop testing)
        (void)rec_.spawn();
    }
}

void Visualizer::flush() {
    // Rerun SDK handles flushing automatically, but we can force it if needed
    // currently rec_.flush() is not exposed in C++ SDK 0.15 directly on RecordingStream in the same way,
    // but the destructor handles it.
    // However, if there's a specific flush mechanism needed we can add it.
    // For now, no-op or rely on SDK defaults.
}

void Visualizer::logLandmarks(const LandmarkMap &landmarks) {
  if (!enabled_) return;
  std::vector<rerun::Position3D> positions;
  std::vector<std::string> labels;

  for (const auto &[id, point] : landmarks.getAllLandmarks()) {
    positions.push_back(
        {static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)});
    labels.push_back("Tag " + std::to_string(id));
  }

  if (!positions.empty()) {
    rec_.log_static("world/landmarks",
                    rerun::Points3D(positions)
                        .with_colors({rerun::Color(255, 165, 0)}) // Orange
                        .with_radii({0.1f})
                        .with_labels(labels));
  }
}

void Visualizer::logPose(const PoseEstimate &pose, size_t pose_idx) {
  if (!enabled_) return;
  rec_.log("world/robot",
           rerun::Transform3D::from_translation_rotation(
               {static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f},
               rerun::Rotation3D(rerun::datatypes::RotationAxisAngle(
                   rerun::datatypes::Vec3D{0.0f, 0.0f, 1.0f},
                   rerun::datatypes::Angle::radians(
                       static_cast<float>(pose.theta))))));

  float radius = 0.15f;
  if (pose.has_covariance) {
    double cov_xx = pose.covariance[0];
    double cov_xy = pose.covariance[1];
    double cov_yy = pose.covariance[4];
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    double discriminant = std::sqrt(std::max(trace * trace / 4.0 - det, 0.0));
    double lambda1 = trace / 2.0 + discriminant;
    radius = static_cast<float>(std::sqrt(std::max(lambda1, 0.0)));
  }

  std::vector<rerun::Position3D> circle_points;
  const int num_points = 32;
  for (int i = 0; i <= num_points; i++) {
    double t = 2.0 * M_PI * i / num_points;
    double x = radius * std::cos(t);
    double y = radius * std::sin(t);
    circle_points.push_back(
        {static_cast<float>(x), static_cast<float>(y), 0.0f});
  }

  rec_.log("world/robot/position",
           rerun::LineStrips3D({circle_points})
               .with_colors({rerun::Color(0, 255, 0)}) // Green
               .with_radii({0.05f}));

  // Log heading as an arrow
  float arrow_len = 0.5f;

  rec_.log("world/robot/heading",
           rerun::Arrows3D::from_vectors(
               {{static_cast<float>(arrow_len), 0.0f, 0.0f}})
               .with_origins({{0.0f, 0.0f, 0.0f}})
               .with_colors({rerun::Color(0, 255, 0)}));

  logUncertaintyEllipse(pose, pose_idx);
}

void Visualizer::logTrajectory(const std::vector<PoseEstimate> &trajectory) {
  if (!enabled_ || trajectory.empty()) {
    return;
  }

  std::vector<rerun::Position3D> points;
  points.reserve(trajectory.size());

  for (const auto &pose : trajectory) {
    points.push_back(
        {static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f});
  }

  rec_.log("world/trajectory", rerun::LineStrips3D({points}).with_colors(
                                   {rerun::Color(0, 128, 255)})); // Blue
}

void Visualizer::logBearingMeasurement(const PoseEstimate &pose,
                                       const gtsam::Point2 &landmark,
                                       int32_t tag_id, double bearing_rad) {
  if (!enabled_) return;
  (void)landmark;

  // Draw a fixed-length ray from the robot in the measured bearing direction.
  const float arrow_len = 0.75f;
  double global_bearing = pose.theta + bearing_rad;

  rec_.log("world/measurements/bearing_ray_" + std::to_string(tag_id),
           rerun::Arrows3D::from_vectors(
               {{static_cast<float>(arrow_len * std::cos(global_bearing)),
                 static_cast<float>(arrow_len * std::sin(global_bearing)), 0.0f}})
               .with_origins({{static_cast<float>(pose.x),
                               static_cast<float>(pose.y), 0.0f}})
               .with_colors({rerun::Color(255, 255, 0, 200)})); // Yellow, opaque
}

void Visualizer::logLandmarkRays(
    const LandmarkMap &landmarks, const PoseEstimate &pose, int64_t step,
    const std::unordered_set<int32_t> &visible_tags) {
  if (!enabled_) return;
  (void)visible_tags;
  rec_.set_time_sequence("step", step);
  for (const auto &[id, point] : landmarks.getAllLandmarks()) {
    std::vector<rerun::Position3D> line = {
        {static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f},
        {static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)}};

    rec_.log("world/measurements/bearing_ray_" + std::to_string(id),
             rerun::LineStrips3D({line})
                 .with_colors({rerun::Color(255, 0, 0, 255)}) // Red, opaque
                 .with_radii({0.05f}));
  }
}

void Visualizer::logTimeSeriesMetrics(
    int64_t step, const PoseEstimate &estimate, const PoseEstimate &true_pose,
    const PoseEstimate &odom_pose, const PoseEstimate &ekf_pose,
    const PoseEstimate &post_estimate, const PoseEstimate &post_ekf,
    double position_error, double odom_error, double ekf_error,
    double post_position_error, double post_ekf_error) {
  
  if (!enabled_) return;
  rec_.set_time_sequence("step", step);

  if (step == 0) {
    estimate_path_.clear();
    true_path_.clear();
    ekf_path_.clear();
    odom_path_.clear();
    post_estimate_path_.clear();
    post_ekf_path_.clear();
  }

  estimate_path_.push_back(
      {static_cast<float>(estimate.x), static_cast<float>(estimate.y), 0.0f});
  true_path_.push_back(
      {static_cast<float>(true_pose.x), static_cast<float>(true_pose.y), 0.0f});
  ekf_path_.push_back(
      {static_cast<float>(ekf_pose.x), static_cast<float>(ekf_pose.y), 0.0f});
  odom_path_.push_back(
      {static_cast<float>(odom_pose.x), static_cast<float>(odom_pose.y), 0.0f});
  post_estimate_path_.push_back({static_cast<float>(post_estimate.x),
                                 static_cast<float>(post_estimate.y), 0.0f});
  post_ekf_path_.push_back(
      {static_cast<float>(post_ekf.x), static_cast<float>(post_ekf.y), 0.0f});
  const size_t max_points = 300;
  if (estimate_path_.size() > max_points) {
    estimate_path_.erase(estimate_path_.begin(),
                         estimate_path_.begin() + (estimate_path_.size() - max_points));
  }
  if (true_path_.size() > max_points) {
    true_path_.erase(true_path_.begin(),
                     true_path_.begin() + (true_path_.size() - max_points));
  }
  if (ekf_path_.size() > max_points) {
    ekf_path_.erase(ekf_path_.begin(),
                    ekf_path_.begin() + (ekf_path_.size() - max_points));
  }
  if (odom_path_.size() > max_points) {
    odom_path_.erase(odom_path_.begin(),
                     odom_path_.begin() + (odom_path_.size() - max_points));
  }
  if (post_estimate_path_.size() > max_points) {
    post_estimate_path_.erase(
        post_estimate_path_.begin(),
        post_estimate_path_.begin() + (post_estimate_path_.size() - max_points));
  }
  if (post_ekf_path_.size() > max_points) {
    post_ekf_path_.erase(
        post_ekf_path_.begin(),
        post_ekf_path_.begin() + (post_ekf_path_.size() - max_points));
  }

  rerun::LineStrip3D estimate_strip(estimate_path_);
  rerun::LineStrip3D true_strip(true_path_);
  rerun::LineStrip3D ekf_strip(ekf_path_);
  rerun::LineStrip3D odom_strip(odom_path_);
  rerun::LineStrip3D post_estimate_strip(post_estimate_path_);
  rerun::LineStrip3D post_ekf_strip(post_ekf_path_);

  rec_.log("world/trajectory/estimate",
           rerun::LineStrips3D(estimate_strip)
               .with_colors({rerun::Color(0, 128, 255)})); // Blue
  rec_.log("world/trajectory/true",
           rerun::LineStrips3D(true_strip)
               .with_colors({rerun::Color(255, 0, 0)})); // Red
  rec_.log("world/trajectory/ekf",
           rerun::LineStrips3D(ekf_strip)
               .with_colors({rerun::Color(0, 200, 0)})); // Green
  rec_.log("world/trajectory/odom",
           rerun::LineStrips3D(odom_strip)
               .with_colors({rerun::Color(255, 165, 0)})); // Orange
  rec_.log("world/trajectory/post_estimate",
           rerun::LineStrips3D(post_estimate_strip)
               .with_colors({rerun::Color(153, 102, 255)})); // Violet
  rec_.log("world/trajectory/post_ekf",
           rerun::LineStrips3D(post_ekf_strip)
               .with_colors({rerun::Color(0, 200, 200)})); // Cyan

  rec_.log("world/true_robot/position",
           rerun::Points3D(
               {{static_cast<float>(true_pose.x), static_cast<float>(true_pose.y), 0.0f}})
               .with_colors({rerun::Color(255, 0, 0)}) // Red
               .with_radii({0.12f}));

  rec_.log("metrics/error/position_estimate", rerun::Scalars(position_error));
  rec_.log("metrics/error/position_odom", rerun::Scalars(odom_error));
  rec_.log("metrics/error/position_ekf", rerun::Scalars(ekf_error));
  rec_.log("metrics/error/position_post_estimate",
           rerun::Scalars(post_position_error));
  rec_.log("metrics/error/position_post_ekf", rerun::Scalars(post_ekf_error));
  rec_.log("metrics/error/x", rerun::Scalars(estimate.x - true_pose.x));
  rec_.log("metrics/error/y", rerun::Scalars(estimate.y - true_pose.y));
  rec_.log("metrics/error/theta",
           rerun::Scalars(estimate.theta - true_pose.theta));

  rec_.log("metrics/position/estimate_x", rerun::Scalars(estimate.x));
  rec_.log("metrics/position/estimate_y", rerun::Scalars(estimate.y));
  rec_.log("metrics/position/true_x", rerun::Scalars(true_pose.x));
  rec_.log("metrics/position/true_y", rerun::Scalars(true_pose.y));
  rec_.log("metrics/position/odom_x", rerun::Scalars(odom_pose.x));
  rec_.log("metrics/position/odom_y", rerun::Scalars(odom_pose.y));
  rec_.log("metrics/position/ekf_x", rerun::Scalars(ekf_pose.x));
  rec_.log("metrics/position/ekf_y", rerun::Scalars(ekf_pose.y));
  rec_.log("metrics/position/post_estimate_x",
           rerun::Scalars(post_estimate.x));
  rec_.log("metrics/position/post_estimate_y",
           rerun::Scalars(post_estimate.y));
  rec_.log("metrics/position/post_ekf_x", rerun::Scalars(post_ekf.x));
  rec_.log("metrics/position/post_ekf_y", rerun::Scalars(post_ekf.y));
  rec_.log("metrics/position/estimate_theta", rerun::Scalars(estimate.theta));
  rec_.log("metrics/position/true_theta", rerun::Scalars(true_pose.theta));
  rec_.log("metrics/position/odom_theta", rerun::Scalars(odom_pose.theta));
  rec_.log("metrics/position/ekf_theta", rerun::Scalars(ekf_pose.theta));
  rec_.log("metrics/position/post_estimate_theta",
           rerun::Scalars(post_estimate.theta));
  rec_.log("metrics/position/post_ekf_theta",
           rerun::Scalars(post_ekf.theta));
}

void Visualizer::logCamera(const gtsam::Pose3 &camera_pose,
                           const CameraModel &intrinsics) {
  if (!enabled_) return;
  auto axis_angle = camera_pose.rotation().axisAngle();
  gtsam::Point3 axis = axis_angle.first.point3();

  rec_.log("world/robot/turret/camera",
           rerun::Transform3D::from_translation_rotation(
               {static_cast<float>(camera_pose.x()),
                static_cast<float>(camera_pose.y()),
                static_cast<float>(camera_pose.z())},
               rerun::Rotation3D(rerun::datatypes::RotationAxisAngle(
                   rerun::datatypes::Vec3D{static_cast<float>(axis.x()),
                                           static_cast<float>(axis.y()),
                                           static_cast<float>(axis.z())},
                   rerun::datatypes::Angle::radians(
                       static_cast<float>(axis_angle.second))))));

  float width =
      static_cast<float>(std::max(1.0, intrinsics.cx * 2.0));
  float height =
      static_cast<float>(std::max(1.0, intrinsics.cy * 2.0));

  rec_.log("world/robot/turret/camera/frustum",
           rerun::Pinhole::from_focal_length_and_resolution(
               {static_cast<float>(intrinsics.fx),
                static_cast<float>(intrinsics.fy)},
               {width, height}));
}

void Visualizer::logTagCornerRays(
    int32_t tag_id, const gtsam::Pose3 &camera_pose,
    const std::vector<gtsam::Point3> &corners_world) {
  if (!enabled_ || corners_world.empty()) {
    return;
  }

  std::vector<std::vector<rerun::Position3D>> lines;
  lines.reserve(corners_world.size());

  rerun::Position3D camera_pos{static_cast<float>(camera_pose.x()),
                               static_cast<float>(camera_pose.y()),
                               static_cast<float>(camera_pose.z())};

  for (const auto &corner : corners_world) {
    lines.push_back({camera_pos,
                     {static_cast<float>(corner.x()),
                      static_cast<float>(corner.y()),
                      static_cast<float>(corner.z())}});
  }

  rec_.log("world/measurements/tag_corners/" + std::to_string(tag_id),
           rerun::LineStrips3D(lines)
               .with_colors({rerun::Color(0, 255, 255, 200)}) // Cyan
               .with_radii({0.02f}));
}

void Visualizer::logTagCornerComparison(int32_t tag_id,
    const std::vector<std::pair<double,double>>& detected_px,
    const std::vector<std::pair<double,double>>& predicted_px) {
    if (!enabled_) return;

    std::vector<rerun::Position2D> det_points;
    for(const auto& p : detected_px) det_points.push_back({(float)p.first, (float)p.second});
    rec_.log("world/measurements/tag_" + std::to_string(tag_id) + "/detected_corners",
        rerun::Points2D(det_points).with_colors({rerun::Color(0, 255, 0)}));

    std::vector<rerun::Position2D> pred_points;
    for(const auto& p : predicted_px) pred_points.push_back({(float)p.first, (float)p.second});
    rec_.log("world/measurements/tag_" + std::to_string(tag_id) + "/predicted_corners",
        rerun::Points2D(pred_points).with_colors({rerun::Color(255, 0, 0)}));

    // Draw lines between corresponding corners
    if (detected_px.size() == predicted_px.size()) {
        std::vector<std::vector<rerun::Position2D>> lines;
        for (size_t i = 0; i < detected_px.size(); ++i) {
            lines.push_back({
                {(float)detected_px[i].first, (float)detected_px[i].second},
                {(float)predicted_px[i].first, (float)predicted_px[i].second}
            });
        }
        rec_.log("world/measurements/tag_" + std::to_string(tag_id) + "/reprojection_error",
            rerun::LineStrips2D(lines).with_colors({rerun::Color(255, 255, 0)}));
    }
}

void Visualizer::logOdometryDelta(const gtsam::Pose2& delta, double timestamp) {
    if (!enabled_) return;
    (void)timestamp;
    // Log as arrow at origin? Or accum? 
    // Just logging magnitude/direction as vector at origin for debug
     rec_.log("world/measurements/odometry_deltas",
           rerun::Arrows3D::from_vectors(
               {{static_cast<float>(delta.x()), static_cast<float>(delta.y()), 0.0f}})
               .with_origins({{0.0f, 0.0f, 0.0f}})
               .with_colors({rerun::Color(255, 165, 0)})); // Orange
}

void Visualizer::logCoordinateFrames(const gtsam::Pose2& robot_pose,
        double turret_yaw, const gtsam::Pose3& camera_extrinsics) {
    if (!enabled_) return;
    
    // Robot Frame
    rec_.log("world/robot",
           rerun::Transform3D::from_translation_rotation(
               {static_cast<float>(robot_pose.x()), static_cast<float>(robot_pose.y()), 0.0f},
               rerun::Rotation3D(rerun::datatypes::RotationAxisAngle(
                   rerun::datatypes::Vec3D{0.0f, 0.0f, 1.0f},
                   rerun::datatypes::Angle::radians(
                       static_cast<float>(robot_pose.theta()))))));
                       
    // Turret Frame (relative to robot)
    rec_.log("world/robot/turret",
           rerun::Transform3D::from_translation_rotation(
               {0.0f, 0.0f, 0.0f},
               rerun::Rotation3D(rerun::datatypes::RotationAxisAngle(
                   rerun::datatypes::Vec3D{0.0f, 0.0f, 1.0f},
                   rerun::datatypes::Angle::radians(
                       static_cast<float>(turret_yaw))))));

     // Camera Frame (relative to turret)
     // Extract rotation axis/angle from extrinsics
     auto axis_angle = camera_extrinsics.rotation().axisAngle();
     gtsam::Point3 axis = axis_angle.first.point3();
     
     rec_.log("world/robot/turret/camera",
           rerun::Transform3D::from_translation_rotation(
               {static_cast<float>(camera_extrinsics.x()), 
                static_cast<float>(camera_extrinsics.y()), 
                static_cast<float>(camera_extrinsics.z())},
               rerun::Rotation3D(rerun::datatypes::RotationAxisAngle(
                   rerun::datatypes::Vec3D{static_cast<float>(axis.x()),
                                           static_cast<float>(axis.y()),
                                           static_cast<float>(axis.z())},
                   rerun::datatypes::Angle::radians(
                       static_cast<float>(axis_angle.second))))));
}

void Visualizer::logUncertaintyEllipse(const PoseEstimate &pose,
                                       size_t /*pose_idx*/) {
  if (!enabled_ || !pose.has_covariance) {
    return;
  }

  // Extract 2x2 position covariance
  double cov_xx = pose.covariance[0];
  double cov_xy = pose.covariance[1];
  double cov_yy = pose.covariance[4];

  // Compute eigenvalues for ellipse axes
  double trace = cov_xx + cov_yy;
  double det = cov_xx * cov_yy - cov_xy * cov_xy;
  double discriminant = std::sqrt(std::max(trace * trace / 4.0 - det, 0.0));

  double lambda1 = trace / 2.0 + discriminant;
  double lambda2 = trace / 2.0 - discriminant;

  // 2-sigma ellipse (95% confidence)
  double a = 2.0 * std::sqrt(std::max(lambda1, 0.0));
  double b = 2.0 * std::sqrt(std::max(lambda2, 0.0));

  // Ellipse orientation
  double angle = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);

  rec_.log("world/robot/uncertainty",
           rerun::Ellipsoids3D::from_centers_and_half_sizes(
               {rerun::components::Translation3D(
                   0.0f, 0.0f, 0.0f)},
               {rerun::components::HalfSize3D(
                   static_cast<float>(a),
                   static_cast<float>(b),
                   0.01f)})
               .with_rotation_axis_angles(
                   {rerun::components::RotationAxisAngle(
                       rerun::datatypes::RotationAxisAngle(
                           rerun::datatypes::Vec3D{0.0f, 0.0f, 1.0f},
                           rerun::datatypes::Angle::radians(
                               static_cast<float>(angle))))})
               .with_colors(
                   {rerun::Color(0, 255, 0, 128)})); // Green, semi-transparent
}

} // namespace decode

#endif // DECODE_ENABLE_RERUN
