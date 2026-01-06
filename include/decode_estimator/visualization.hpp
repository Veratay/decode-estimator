#pragma once

#include "landmark_map.hpp"
#include "types.hpp"

#include <gtsam/geometry/Point2.h>

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#if DECODE_ENABLE_RERUN
#include <rerun.hpp>
#endif

namespace decode {

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
    ~Visualizer();

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
                              double position_error,
                              double odom_error,
                              double ekf_error);

    /// Log uncertainty ellipse at current pose
    void logUncertaintyEllipse(const PoseEstimate& pose, size_t pose_idx);

private:
    rerun::RecordingStream rec_;
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
    ~Visualizer() = default;

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
                              double /*position_error*/,
                              double /*odom_error*/,
                              double /*ekf_error*/) {}
    void logUncertaintyEllipse(const PoseEstimate& /*pose*/, size_t /*pose_idx*/) {}
};

#endif

} // namespace decode
