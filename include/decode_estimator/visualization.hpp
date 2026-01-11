#pragma once

#include "camera_model.hpp"
#include "landmark_map.hpp"
#include "types.hpp"

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#if DECODE_ENABLE_RERUN
#include <rerun.hpp>
#endif

namespace decode {

struct VisualizationConfig {
    bool enabled = false;
    bool stream_to_viewer = true;
    std::string stream_url = "rerun://localhost:9876";
    std::string save_path = "";
    std::string app_id = "decode_estimator";
};

#if DECODE_ENABLE_RERUN

/**
 * @brief Visualization using Rerun SDK
 *
 * Provides real-time visualization of robot pose, trajectory, landmarks,
 * and bearing measurements for debugging and monitoring.
 */
class Visualizer {
public:
    explicit Visualizer(const std::string& app_id);
    explicit Visualizer(const VisualizationConfig& config);
    ~Visualizer();

    void setEnabled(bool enabled);
    bool isEnabled() const;
    void configure(const VisualizationConfig& config);
    void flush();

    /// Log landmark positions (call once at initialization)
    void logLandmarks(const LandmarkMap& landmarks);

    /// Log current robot pose
    void logPose(const PoseEstimate& pose, size_t pose_idx);

    /// Log full trajectory (all poses)
    void logTrajectory(const std::vector<PoseEstimate>& trajectory);

    /// Log bearing measurement (ray from robot to landmark)
    void logBearingMeasurement(const PoseEstimate& pose,
                                const gtsam::Point2& landmark,
                                int32_t tag_id,
                                double bearing_rad);

    /// Log rays from robot to landmarks (use NaN for out-of-view rays)
    void logLandmarkRays(const LandmarkMap& landmarks,
                         const PoseEstimate& pose,
                         int64_t step,
                         const std::unordered_set<int32_t>& visible_tags);

    /// Log time series metrics (errors, estimated/true positions)
    void logTimeSeriesMetrics(int64_t step,
                              const PoseEstimate& estimate,
                              const PoseEstimate& true_pose,
                              const PoseEstimate& odom_pose,
                              const PoseEstimate& ekf_pose,
                              const PoseEstimate& post_estimate,
                              const PoseEstimate& post_ekf,
                              double position_error,
                              double odom_error,
                              double ekf_error,
                              double post_position_error,
                              double post_ekf_error);

    /// Log camera intrinsics/extrinsics
    void logCamera(const gtsam::Pose3& camera_pose,
                   const CameraModel& intrinsics);

    /// Log rays from camera to detected tag corners
    void logTagCornerRays(int32_t tag_id,
                          const gtsam::Pose3& camera_pose,
                          const std::vector<gtsam::Point3>& corners_world);

    void logTagCornerComparison(int32_t tag_id,
        const std::vector<std::pair<double,double>>& detected_px,
        const std::vector<std::pair<double,double>>& predicted_px);
    
    void logOdometryDelta(const gtsam::Pose2& delta, double timestamp);
    
    void logCoordinateFrames(const gtsam::Pose2& robot_pose,
        double turret_yaw, const gtsam::Pose3& camera_extrinsics);

    /// Log uncertainty ellipse at current pose
    void logUncertaintyEllipse(const PoseEstimate& pose, size_t pose_idx);

private:
    rerun::RecordingStream rec_;
    bool enabled_ = true;
    std::vector<rerun::Position3D> estimate_path_;
    std::vector<rerun::Position3D> true_path_;
    std::vector<rerun::Position3D> ekf_path_;
    std::vector<rerun::Position3D> odom_path_;
    std::vector<rerun::Position3D> post_estimate_path_;
    std::vector<rerun::Position3D> post_ekf_path_;
};

#else

/**
 * @brief Stub implementation when Rerun is disabled
 *
 * All methods are no-ops to avoid conditional compilation throughout the codebase.
 */
class Visualizer {
public:
    explicit Visualizer(const std::string& /*app_id*/) {}
    explicit Visualizer(const VisualizationConfig& /*config*/) {}
    ~Visualizer() = default;

    void setEnabled(bool /*enabled*/) {}
    bool isEnabled() const { return false; }
    void configure(const VisualizationConfig& /*config*/) {}
    void flush() {}

    void logLandmarks(const LandmarkMap& /*landmarks*/) {}
    void logPose(const PoseEstimate& /*pose*/, size_t /*pose_idx*/) {}
    void logTrajectory(const std::vector<PoseEstimate>& /*trajectory*/) {}
    void logBearingMeasurement(const PoseEstimate& /*pose*/,
                                const gtsam::Point2& /*landmark*/,
                                int32_t /*tag_id*/,
                                double /*bearing_rad*/) {}
    void logLandmarkRays(const LandmarkMap& /*landmarks*/,
                         const PoseEstimate& /*pose*/,
                         int64_t /*step*/,
                         const std::unordered_set<int32_t>& /*visible_tags*/) {}
    void logTimeSeriesMetrics(int64_t /*step*/,
                              const PoseEstimate& /*estimate*/,
                              const PoseEstimate& /*true_pose*/,
                              const PoseEstimate& /*odom_pose*/,
                              const PoseEstimate& /*ekf_pose*/,
                              const PoseEstimate& /*post_estimate*/,
                              const PoseEstimate& /*post_ekf*/,
                              double /*position_error*/,
                              double /*odom_error*/,
                              double /*ekf_error*/,
                              double /*post_position_error*/,
                              double /*post_ekf_error*/) {}
    void logCamera(const gtsam::Pose3& /*camera_pose*/,
                   const CameraModel& /*intrinsics*/) {}
    void logTagCornerRays(int32_t /*tag_id*/,
                          const gtsam::Pose3& /*camera_pose*/,
                          const std::vector<gtsam::Point3>& /*corners_world*/) {}
    void logTagCornerComparison(int32_t /*tag_id*/,
        const std::vector<std::pair<double,double>>& /*detected_px*/,
        const std::vector<std::pair<double,double>>& /*predicted_px*/) {}
    void logOdometryDelta(const gtsam::Pose2& /*delta*/, double /*timestamp*/) {}
    void logCoordinateFrames(const gtsam::Pose2& /*robot_pose*/,
        double /*turret_yaw*/, const gtsam::Pose3& /*camera_extrinsics*/) {}
    void logUncertaintyEllipse(const PoseEstimate& /*pose*/, size_t /*pose_idx*/) {}
};

#endif

} // namespace decode
