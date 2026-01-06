#include "decode_estimator/visualization.hpp"

#if DECODE_ENABLE_RERUN

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace decode {

Visualizer::Visualizer(const std::string &app_id) : rec_(app_id) {
  // Spawn the Rerun viewer
  rec_.spawn().exit_on_failure();
}

Visualizer::~Visualizer() = default;

void Visualizer::logLandmarks(const LandmarkMap &landmarks) {
  std::vector<rerun::Position3D> positions;
  std::vector<std::string> labels;

  for (const auto &[id, point] : landmarks.getAllLandmarks()) {
    positions.push_back(
        {static_cast<float>(point.x()), static_cast<float>(point.y()), 0.0f});
    labels.push_back("Tag " + std::to_string(id));
  }

  if (!positions.empty()) {
    rec_.log_static("world/landmarks",
                    rerun::Points3D(positions)
                        .with_colors({rerun::Color(255, 165, 0)}) // Orange
                        .with_radii({0.1f}));
  }
}

void Visualizer::logPose(const PoseEstimate &pose, size_t pose_idx) {
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
  if (trajectory.empty()) {
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
  (void)visible_tags;
  rec_.set_time_sequence("step", step);
  for (const auto &[id, point] : landmarks.getAllLandmarks()) {
    std::vector<rerun::Position3D> line = {
        {static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f},
        {static_cast<float>(point.x()), static_cast<float>(point.y()), 0.0f}};

    rec_.log("world/measurements/bearing_ray_" + std::to_string(id),
             rerun::LineStrips3D({line})
                 .with_colors({rerun::Color(255, 0, 0, 255)}) // Red, opaque
                 .with_radii({0.05f}));
  }
}

void Visualizer::logTimeSeriesMetrics(
    int64_t step, const PoseEstimate &estimate, const PoseEstimate &true_pose,
    const PoseEstimate &odom_pose, double position_error, double odom_error) {
  rec_.set_time_sequence("step", step);

  rec_.log("world/true_robot/position",
           rerun::Points3D(
               {{static_cast<float>(true_pose.x), static_cast<float>(true_pose.y), 0.0f}})
               .with_colors({rerun::Color(255, 0, 0)}) // Red
               .with_radii({0.12f}));

  rec_.log("metrics/error/position_estimate", rerun::Scalars(position_error));
  rec_.log("metrics/error/position_odom", rerun::Scalars(odom_error));
  rec_.log("metrics/error/x", rerun::Scalars(estimate.x - true_pose.x));
  rec_.log("metrics/error/y", rerun::Scalars(estimate.y - true_pose.y));

  rec_.log("metrics/position/estimate_x", rerun::Scalars(estimate.x));
  rec_.log("metrics/position/estimate_y", rerun::Scalars(estimate.y));
  rec_.log("metrics/position/true_x", rerun::Scalars(true_pose.x));
  rec_.log("metrics/position/true_y", rerun::Scalars(true_pose.y));
  rec_.log("metrics/position/odom_x", rerun::Scalars(odom_pose.x));
  rec_.log("metrics/position/odom_y", rerun::Scalars(odom_pose.y));
}

void Visualizer::logUncertaintyEllipse(const PoseEstimate &pose,
                                       size_t /*pose_idx*/) {
  if (!pose.has_covariance) {
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
