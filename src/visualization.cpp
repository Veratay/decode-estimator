#include "decode_estimator/visualization.hpp"

#if DECODE_ENABLE_RERUN

#include <cmath>

namespace decode {

Visualizer::Visualizer(const std::string& app_id) : rec_(app_id) {
    // Spawn the Rerun viewer
    rec_.spawn().exit_on_failure();
}

Visualizer::~Visualizer() = default;

void Visualizer::logLandmarks(const LandmarkMap& landmarks) {
    std::vector<rerun::Position3D> positions;
    std::vector<std::string> labels;

    for (const auto& [id, point] : landmarks.getAllLandmarks()) {
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

void Visualizer::logPose(const PoseEstimate& pose, size_t pose_idx) {
    // Log pose as a point
    rec_.log("world/robot/position",
             rerun::Points3D({{static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f}})
                 .with_colors({rerun::Color(0, 255, 0)}) // Green
                 .with_radii({0.15f}));

    // Log heading as an arrow
    float arrow_len = 0.5f;

    rec_.log("world/robot/heading",
             rerun::Arrows3D::from_vectors(
                 {{static_cast<float>(arrow_len * std::cos(pose.theta)),
                   static_cast<float>(arrow_len * std::sin(pose.theta)), 0.0f}})
                 .with_origins({{static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f}})
                 .with_colors({rerun::Color(0, 255, 0)}));
}

void Visualizer::logTrajectory(const std::vector<PoseEstimate>& trajectory) {
    if (trajectory.empty()) {
        return;
    }

    std::vector<rerun::Position3D> points;
    points.reserve(trajectory.size());

    for (const auto& pose : trajectory) {
        points.push_back(
            {static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f});
    }

    rec_.log("world/trajectory",
             rerun::LineStrips3D({points}).with_colors({rerun::Color(0, 128, 255)})); // Blue
}

void Visualizer::logBearingMeasurement(const PoseEstimate& pose,
                                        const gtsam::Point2& landmark,
                                        int32_t tag_id) {
    // Draw line from robot to landmark
    std::vector<rerun::Position3D> line = {
        {static_cast<float>(pose.x), static_cast<float>(pose.y), 0.0f},
        {static_cast<float>(landmark.x()), static_cast<float>(landmark.y()), 0.0f}};

    rec_.log("world/measurements/bearing_" + std::to_string(tag_id),
             rerun::LineStrips3D({line})
                 .with_colors({rerun::Color(255, 255, 0, 128)})); // Yellow, semi-transparent
}

void Visualizer::logUncertaintyEllipse(const PoseEstimate& pose, size_t /*pose_idx*/) {
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

    // Generate ellipse points
    std::vector<rerun::Position3D> ellipse_points;
    const int num_points = 32;
    for (int i = 0; i <= num_points; i++) {
        double t = 2.0 * M_PI * i / num_points;
        double ex = a * std::cos(t);
        double ey = b * std::sin(t);

        // Rotate and translate
        double x = pose.x + ex * std::cos(angle) - ey * std::sin(angle);
        double y = pose.y + ex * std::sin(angle) + ey * std::cos(angle);

        ellipse_points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0f});
    }

    rec_.log("world/robot/uncertainty",
             rerun::LineStrips3D({ellipse_points})
                 .with_colors({rerun::Color(0, 255, 0, 128)})); // Green, semi-transparent
}

} // namespace decode

#endif // DECODE_ENABLE_RERUN
