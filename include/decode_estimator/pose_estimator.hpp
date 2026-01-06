#pragma once

#include "bearing_factor.hpp"
#include "landmark_map.hpp"
#include "types.hpp"
#include "visualization.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <vector>

namespace decode {

/// Configuration for the pose estimator
struct EstimatorConfig {
    // iSAM2 parameters
    double relinearize_threshold = 0.01; ///< Threshold for relinearization
    int relinearize_skip = 1;            ///< Relinearize every N updates
    bool enable_partial_relinearization = true;

    // Noise model parameters (1-sigma values)
    double prior_sigma_xy = 0.05;    ///< Prior position uncertainty (meters)
    double prior_sigma_theta = 0.02; ///< Prior heading uncertainty (radians)

    double odom_sigma_xy = 0.02;    ///< Odometry position uncertainty per update (meters)
    double odom_sigma_theta = 0.01; ///< Odometry heading uncertainty per update (radians)

    /// If true, compacts odometry between vision updates into a single factor
    bool compact_odometry = true;

    /// Default bearing uncertainty if not provided per measurement (radians, ~3 degrees)
    double default_bearing_sigma = 0.05;
    /// Default distance uncertainty if not provided per measurement (meters)
    double default_distance_sigma = 0.2;

    // Visualization
    bool enable_visualization = false;
    std::string visualization_app_id = "decode_estimator";
};

/**
 * @brief 2D robot pose estimator using GTSAM iSAM2
 *
 * Fuses odometry (BetweenFactor) with bearing and distance measurements to known
 * AprilTag landmarks (custom factors) for accurate localization that corrects
 * odometry drift.
 *
 * Thread-safe: all public methods are mutex-protected.
 */
class PoseEstimator {
public:
    explicit PoseEstimator(const EstimatorConfig& config = EstimatorConfig());
    ~PoseEstimator();

    // Non-copyable, non-movable (due to mutex)
    PoseEstimator(const PoseEstimator&) = delete;
    PoseEstimator& operator=(const PoseEstimator&) = delete;

    /**
     * @brief Initialize with known landmarks and starting pose
     * @param landmarks  Vector of known AprilTag positions
     * @param initial_x  Initial X position (meters)
     * @param initial_y  Initial Y position (meters)
     * @param initial_theta  Initial heading (radians)
     */
    void initialize(const std::vector<Landmark>& landmarks,
                    double initial_x,
                    double initial_y,
                    double initial_theta);

    /// Reset the estimator to uninitialized state
    void reset();

    /**
     * @brief Process odometry measurement
     *
     * Accumulates odometry until the next vision update is available.
     * Call update() after adding vision measurements to incorporate them.
     *
     * @param odom  Odometry measurement (dx, dy, dtheta in robot frame)
     * @return Current predicted pose (before optimization)
     */
    PoseEstimate processOdometry(const OdometryMeasurement& odom);

    /**
     * @brief Add a bearing measurement for the next update
     * @param bearing  Bearing measurement to known AprilTag
     */
    void addBearingMeasurement(const BearingMeasurement& bearing);

    /**
     * @brief Add multiple bearing measurements for the next update
     * @param bearings  Vector of bearing measurements
     */
    void addBearingMeasurements(const std::vector<BearingMeasurement>& bearings);

    /**
     * @brief Add a distance measurement for the next update
     * @param distance  Distance measurement to known AprilTag
     */
    void addDistanceMeasurement(const DistanceMeasurement& distance);

    /**
     * @brief Add multiple distance measurements for the next update
     * @param distances  Vector of distance measurements
     */
    void addDistanceMeasurements(const std::vector<DistanceMeasurement>& distances);

    /**
     * @brief Perform iSAM2 update
     *
     * Incorporates pending vision factors and any accumulated odometry since
     * the last vision update, then optimizes and returns the updated estimate.
     *
     * @return Updated pose estimate
     */
    PoseEstimate update();

    /// Get current pose estimate without adding new measurements
    PoseEstimate getCurrentEstimate() const;

    /// Get pose estimate with covariance (more expensive)
    PoseEstimate getCurrentEstimateWithCovariance() const;

    /// Get full trajectory (all poses from X(0) to X(current))
    std::vector<PoseEstimate> getTrajectory() const;

    /// Get current pose index
    size_t getCurrentPoseIndex() const;

    /// Get current horizon size (number of poses in graph)
    size_t getHorizonSize() const;

    /// Check if initialized
    bool isInitialized() const;

    /// Get last solve time (milliseconds)
    double getLastSolveTimeMs() const;

    /// Get average solve time (milliseconds)
    double getAverageSolveTimeMs() const;

    /// Get landmark map (for visualization/debugging)
    const LandmarkMap& getLandmarkMap() const;

    /// Log rays from robot to landmarks (visualization only)
    void logLandmarkRays(const PoseEstimate& pose,
                         const std::unordered_set<int32_t>& visible_tags);

    /// Log metrics as time series data (visualization only)
    void logTimeSeriesMetrics(int64_t step,
                              const PoseEstimate& estimate,
                              const PoseEstimate& true_pose,
                              const PoseEstimate& odom_pose,
                              const PoseEstimate& ekf_pose,
                              double position_error,
                              double odom_error,
                              double ekf_error);

private:
    /// Create noise models from config
    void createNoiseModels();

    /// Create iSAM2 instance with configured parameters
    void createISAM2();

    /// Get symbol for pose at index
    static gtsam::Symbol poseSymbol(size_t idx);

    /// Reset horizon and seed with a prior at the current pose
    void resetHorizonWithPrior(const gtsam::Pose2& pose,
                               const gtsam::SharedNoiseModel& prior_noise);

    /// Update visualization (no-op if disabled)
    void visualize();

private:
    static constexpr size_t kHorizonCapacity = 2000;

    EstimatorConfig config_;
    bool initialized_ = false;

    // GTSAM components
    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::NonlinearFactorGraph pending_graph_;
    gtsam::Values pending_values_;

    // Landmark map
    LandmarkMap landmark_map_;

    // State tracking
    size_t current_pose_idx_ = 0;
    gtsam::Pose2 current_pose_;
    gtsam::Pose2 last_solved_pose_;
    double current_timestamp_ = 0.0;
    gtsam::Pose2 pending_odom_delta_;
    size_t pending_odom_steps_ = 0;

    // Noise models (cached for performance)
    gtsam::SharedNoiseModel prior_noise_;
    gtsam::SharedNoiseModel odom_noise_;

    // Pending bearing measurements for next update
    std::vector<BearingMeasurement> pending_bearings_;
    std::vector<DistanceMeasurement> pending_distances_;

    // Thread safety
    mutable std::mutex mutex_;

    // Visualization (defined in visualization.hpp)
    std::unique_ptr<Visualizer> visualizer_;

    // Solve timing (milliseconds)
    double last_solve_ms_ = 0.0;
    double total_solve_ms_ = 0.0;
    size_t solve_count_ = 0;
};

} // namespace decode
