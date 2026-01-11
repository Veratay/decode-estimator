#pragma once

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

enum class RobustLossType {
    Huber,
    Tukey,
    Cauchy
};

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

    /// Default pixel uncertainty (pixels)
    double default_pixel_sigma = 1.0;

    /// Robust loss for tag measurements
    bool enable_robust_tag_loss = false;
    RobustLossType robust_tag_loss = RobustLossType::Huber;
    double robust_tag_loss_k = 1.5;

    /// Tag gating (TagSLAM-inspired)
    bool enable_tag_gating = true;
    double min_tag_area_px = 50.0;
    double max_tag_view_angle_deg = 60.0;

    /// Cheirality check (penalize tags behind camera)
    bool enable_cheirality_check = true;
    double cheirality_sigma = 0.1;
    double min_tag_z_distance = 0.1;

    /// Post-process estimates after vision gap resets
    bool enable_post_process = true;
    double post_process_vision_gap_s = 0.4;
    double post_process_settle_s = 2.0;
    int post_process_settle_updates = 3;

    /// Camera Intrinsics
    double fx = 1000.0;
    double fy = 1000.0;
    double cx = 320.0;
    double cy = 240.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;

    /// Camera Extrinsics (Robot body to Camera)
    double camera_offset_x = 0.0;
    double camera_offset_y = 0.0;
    double camera_offset_z = 0.5;
    double camera_roll = 0.0;
    double camera_pitch = 0.0;
    double camera_yaw = 0.0;

    // Visualization
    bool enable_visualization = false;
    std::string visualization_app_id = "decode_estimator";
};

/**
 * @brief 2D robot pose estimator using GTSAM iSAM2
 *
 * Fuses odometry (BetweenFactor) with AprilTag corner measurements (TagProjectionFactor)
 * to known landmarks for accurate localization that corrects odometry drift.
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
     * @brief Add a tag measurement for the next update
     * @param measurement Tag measurement (pixels)
     */
    void addTagMeasurement(const TagMeasurement& measurement);

    /**
     * @brief Add multiple tag measurements for the next update
     * @param measurements Vector of tag measurements
     */
    void addTagMeasurements(const std::vector<TagMeasurement>& measurements);

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

    /// Get post-processed pose estimate (vision gap smoothing)
    PoseEstimate getPostProcessedEstimate() const;

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

    /// Get last recorded memory usage snapshot
    MemoryUsage getLastMemoryUsage() const;

    /// Get diagnostic counters and timings
    DiagnosticsSnapshot getDiagnosticsSnapshot() const;

    /// Get landmark map (for visualization/debugging)
    const LandmarkMap& getLandmarkMap() const;

    /// Get current camera pose in world frame
    /// Uses last known robot pose, turret yaw, and camera extrinsics
    /// @return Camera pose (position + orientation) in world frame
    gtsam::Pose3 getCurrentCameraPose() const;

    /// Log rays from robot to landmarks (visualization only)
    void logLandmarkRays(const PoseEstimate& pose,
                         const std::unordered_set<int32_t>& visible_tags);

    /// Log metrics as time series data (visualization only)
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
    
    // Visualization Control
    void enableVisualization(bool enabled);
    void configureVisualization(bool stream, const std::string& url_or_path, const std::string& app_id);
    void flushVisualization();
    
    // Helper for debugging/viz
    std::vector<std::pair<double,double>> getPredictedCorners(int32_t tag_id) const;

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

    /// Record process memory usage (caller must hold mutex)
    void recordMemoryUsageLocked();

private:
    static constexpr size_t kHorizonCapacity = 2000;

    EstimatorConfig config_;
    bool initialized_ = false;

    // GTSAM components
    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::NonlinearFactorGraph pending_graph_;
    gtsam::Values pending_values_;

    double last_turret_yaw_rad_ = 0.0;

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

    // Pending measurements for next update
    std::vector<TagMeasurement> pending_tags_;

    // Thread safety
    mutable std::mutex mutex_;

    // Visualization (defined in visualization.hpp)
    std::unique_ptr<Visualizer> visualizer_;

    // Solve timing (milliseconds)
    double last_solve_ms_ = 0.0;
    double total_solve_ms_ = 0.0;
    size_t solve_count_ = 0;

    // Memory usage snapshot
    MemoryUsage last_memory_usage_;

    // Horizon reset diagnostics
    double last_horizon_reset_ms_ = 0.0;
    double last_horizon_cov_ms_ = 0.0;
    size_t last_horizon_reset_pose_index_ = 0;

    // Post-process state for vision resets
    double last_tag_timestamp_ = -1.0;
    bool post_unstable_ = false;
    double unstable_until_timestamp_ = -1.0;
    int settle_updates_remaining_ = 0;
    gtsam::Pose2 last_stable_pose_;
    gtsam::Pose2 odom_since_stable_;

};

} // namespace decode
