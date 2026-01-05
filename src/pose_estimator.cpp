#include "decode_estimator/pose_estimator.hpp"
#include "decode_estimator/visualization.hpp"

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>
#include <stdexcept>

// Use symbol shorthand for poses
using gtsam::symbol_shorthand::X;

namespace decode {

PoseEstimator::PoseEstimator(const EstimatorConfig& config) : config_(config) {
    createNoiseModels();
    createISAM2();

#if DECODE_ENABLE_RERUN
    if (config_.enable_visualization) {
        visualizer_ = std::make_unique<Visualizer>(config_.visualization_app_id);
    }
#endif
}

PoseEstimator::~PoseEstimator() = default;

void PoseEstimator::createNoiseModels() {
    // Prior noise (tight for known initial position)
    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(config_.prior_sigma_xy, config_.prior_sigma_xy, config_.prior_sigma_theta));

    // Odometry noise (per-step uncertainty)
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(config_.odom_sigma_xy, config_.odom_sigma_xy, config_.odom_sigma_theta));
}

void PoseEstimator::createISAM2() {
    gtsam::ISAM2Params params;

    // Relinearization threshold - lower = more accurate but slower
    params.relinearizeThreshold = config_.relinearize_threshold;

    // Relinearize every N updates (1 = every update)
    params.relinearizeSkip = config_.relinearize_skip;

    // Enable relinearization
    params.enableRelinearization = true;

    // Partial relinearization check for early termination
    params.enablePartialRelinearizationCheck = config_.enable_partial_relinearization;

    // Use Cholesky factorization (faster than QR, usually stable)
    params.factorization = gtsam::ISAM2Params::CHOLESKY;

    // Cache linearized factors for faster incremental updates
    params.cacheLinearizedFactors = true;

    // Don't compute nonlinear error (saves time unless debugging)
    params.evaluateNonlinearError = false;

    isam2_ = std::make_unique<gtsam::ISAM2>(params);
}

gtsam::Symbol PoseEstimator::poseSymbol(size_t idx) {
    return X(idx);
}

void PoseEstimator::initialize(const std::vector<Landmark>& landmarks,
                                double initial_x,
                                double initial_y,
                                double initial_theta) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Load landmarks
    landmark_map_.loadFromVector(landmarks);

    // Reset state
    current_pose_idx_ = 0;
    current_pose_ = gtsam::Pose2(initial_x, initial_y, initial_theta);
    current_timestamp_ = 0.0;
    pending_bearings_.clear();

    // Clear any pending factors/values
    pending_graph_.resize(0);
    pending_values_.clear();

    // Recreate iSAM2 to clear previous state
    createISAM2();

    // Add prior factor for initial pose
    pending_graph_.add(
        gtsam::PriorFactor<gtsam::Pose2>(X(0), current_pose_, prior_noise_));
    pending_values_.insert(X(0), current_pose_);

    // Perform initial iSAM2 update
    isam2_->update(pending_graph_, pending_values_);
    pending_graph_.resize(0);
    pending_values_.clear();

    initialized_ = true;

#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        visualizer_->logLandmarks(landmark_map_);
        PoseEstimate est;
        est.x = current_pose_.x();
        est.y = current_pose_.y();
        est.theta = current_pose_.theta();
        est.timestamp = current_timestamp_;
        est.has_covariance = false;
        visualizer_->logPose(est, 0);
    }
#endif
}

void PoseEstimator::reset() {
    std::lock_guard<std::mutex> lock(mutex_);

    initialized_ = false;
    current_pose_idx_ = 0;
    current_pose_ = gtsam::Pose2();
    current_timestamp_ = 0.0;
    pending_bearings_.clear();
    pending_graph_.resize(0);
    pending_values_.clear();
    landmark_map_.clear();

    createISAM2();
}

PoseEstimate PoseEstimator::processOdometry(const OdometryMeasurement& odom) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        throw std::runtime_error("PoseEstimator not initialized");
    }

    // Create relative pose from odometry (in robot body frame)
    gtsam::Pose2 delta(odom.dx, odom.dy, odom.dtheta);

    // Predict new pose by composing current pose with delta
    gtsam::Pose2 predicted = current_pose_.compose(delta);

    // Add BetweenFactor connecting previous pose to new pose
    size_t prev_idx = current_pose_idx_;
    current_pose_idx_++;

    pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
        X(prev_idx), X(current_pose_idx_), delta, odom_noise_));

    // Add initial estimate for new pose
    pending_values_.insert(X(current_pose_idx_), predicted);

    // Update current pose prediction
    current_pose_ = predicted;
    current_timestamp_ = odom.timestamp;

    // Return current predicted estimate
    PoseEstimate estimate;
    estimate.x = current_pose_.x();
    estimate.y = current_pose_.y();
    estimate.theta = current_pose_.theta();
    estimate.timestamp = current_timestamp_;
    estimate.has_covariance = false;

    return estimate;
}

void PoseEstimator::addBearingMeasurement(const BearingMeasurement& bearing) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_bearings_.push_back(bearing);
}

void PoseEstimator::addBearingMeasurements(const std::vector<BearingMeasurement>& bearings) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_bearings_.insert(pending_bearings_.end(), bearings.begin(), bearings.end());
}

PoseEstimate PoseEstimator::update() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        throw std::runtime_error("PoseEstimator not initialized");
    }

    std::unordered_set<int32_t> visible_tags;
    for (const auto& [id, point] : landmark_map_.getAllLandmarks()) {
        (void)point;
        visible_tags.insert(id);
    }

    // Process pending bearing measurements
    for (const auto& bearing : pending_bearings_) {
        auto landmark_opt = landmark_map_.getLandmark(bearing.tag_id);
        if (!landmark_opt) {
            // Unknown tag ID - skip
            continue;
        }

        // Create bearing noise model from measurement uncertainty
        double sigma = (bearing.uncertainty_rad > 0) ? bearing.uncertainty_rad
                                                      : config_.default_bearing_sigma;
        auto bearing_noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma);

        // Add bearing factor to current pose
        pending_graph_.add(BearingToKnownLandmarkFactor(
            X(current_pose_idx_), *landmark_opt, bearing.bearing_rad, bearing_noise));

#if DECODE_ENABLE_RERUN
        if (visualizer_) {
            PoseEstimate est;
            est.x = current_pose_.x();
            est.y = current_pose_.y();
            est.theta = current_pose_.theta();
            est.timestamp = current_timestamp_;
            est.has_covariance = false;
            visualizer_->logBearingMeasurement(est, *landmark_opt, bearing.tag_id,
                                               bearing.bearing_rad);
        }
#endif

        std::cout << "bearing tag=" << bearing.tag_id
                  << " pose=(" << current_pose_.x() << ", " << current_pose_.y()
                  << ", " << current_pose_.theta() << ")"
                  << " landmark=(" << landmark_opt->x() << ", " << landmark_opt->y() << ")"
                  << " bearing_rad=" << bearing.bearing_rad
                  << std::endl;
    }
    pending_bearings_.clear();

    // Perform iSAM2 update
    if (pending_graph_.size() > 0 || pending_values_.size() > 0) {
        isam2_->update(pending_graph_, pending_values_);

        pending_graph_.resize(0);
        pending_values_.clear();
    }

    // Get updated estimate
    gtsam::Values estimate = isam2_->calculateEstimate();
    current_pose_ = estimate.at<gtsam::Pose2>(X(current_pose_idx_));

    PoseEstimate result;
    result.x = current_pose_.x();
    result.y = current_pose_.y();
    result.theta = current_pose_.theta();
    result.timestamp = current_timestamp_;
    result.has_covariance = false;

#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        visualizer_->logLandmarkRays(landmark_map_, result,
                                     static_cast<int64_t>(current_pose_idx_),
                                     visible_tags);
        visualizer_->logPose(result, current_pose_idx_);
    }
#endif

    return result;
}

void PoseEstimator::logTimeSeriesMetrics(int64_t step,
                                         const PoseEstimate& estimate,
                                         const PoseEstimate& true_pose,
                                         const PoseEstimate& odom_pose,
                                         double position_error,
                                         double odom_error) {
    std::lock_guard<std::mutex> lock(mutex_);

#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        visualizer_->logTimeSeriesMetrics(step,
                                          estimate,
                                          true_pose,
                                          odom_pose,
                                          position_error,
                                          odom_error);
    }
#else
    (void)step;
    (void)estimate;
    (void)true_pose;
    (void)odom_pose;
    (void)position_error;
    (void)odom_error;
#endif
}

PoseEstimate PoseEstimator::getCurrentEstimate() const {
    std::lock_guard<std::mutex> lock(mutex_);

    PoseEstimate result;
    result.x = current_pose_.x();
    result.y = current_pose_.y();
    result.theta = current_pose_.theta();
    result.timestamp = current_timestamp_;
    result.has_covariance = false;

    return result;
}

PoseEstimate PoseEstimator::getCurrentEstimateWithCovariance() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        throw std::runtime_error("PoseEstimator not initialized");
    }

    gtsam::Values estimate = isam2_->calculateEstimate();
    gtsam::Pose2 pose = estimate.at<gtsam::Pose2>(X(current_pose_idx_));

    PoseEstimate result;
    result.x = pose.x();
    result.y = pose.y();
    result.theta = pose.theta();
    result.timestamp = current_timestamp_;

    // Compute marginal covariance
    try {
        gtsam::Marginals marginals(isam2_->getFactorsUnsafe(), estimate);
        gtsam::Matrix33 cov = marginals.marginalCovariance(X(current_pose_idx_));

        // Copy to array (row-major)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.covariance[static_cast<size_t>(i * 3 + j)] = cov(i, j);
            }
        }
        result.has_covariance = true;
    } catch (...) {
        result.has_covariance = false;
    }

    return result;
}

std::vector<PoseEstimate> PoseEstimator::getTrajectory() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        return {};
    }

    gtsam::Values estimate = isam2_->calculateEstimate();
    std::vector<PoseEstimate> trajectory;
    trajectory.reserve(current_pose_idx_ + 1);

    for (size_t i = 0; i <= current_pose_idx_; i++) {
        gtsam::Pose2 pose = estimate.at<gtsam::Pose2>(X(i));
        PoseEstimate est;
        est.x = pose.x();
        est.y = pose.y();
        est.theta = pose.theta();
        est.timestamp = 0.0; // Timestamps not stored per-pose
        est.has_covariance = false;
        trajectory.push_back(est);
    }

    return trajectory;
}

size_t PoseEstimator::getCurrentPoseIndex() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_pose_idx_;
}

bool PoseEstimator::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
}

const LandmarkMap& PoseEstimator::getLandmarkMap() const {
    // No lock needed - landmark map is only modified in initialize()
    return landmark_map_;
}

void PoseEstimator::visualize() {
#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        PoseEstimate est;
        est.x = current_pose_.x();
        est.y = current_pose_.y();
        est.theta = current_pose_.theta();
        est.timestamp = current_timestamp_;
        est.has_covariance = false;
        visualizer_->logPose(est, current_pose_idx_);
    }
#endif
}

} // namespace decode
