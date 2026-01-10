#include "decode_estimator/pose_estimator.hpp"
#include "decode_estimator/tag_projection_factor.hpp"
#include "decode_estimator/visualization.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>

#include <array>
#include <chrono>
#include <cmath>
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

void PoseEstimator::resetHorizonWithPrior(const gtsam::Pose2& pose,
                                          const gtsam::SharedNoiseModel& prior_noise) {
    // Clear any pending factors/values
    pending_graph_.resize(0);
    pending_values_.clear();

    // Recreate iSAM2 to clear previous state
    createISAM2();

    // Seed new horizon with prior
    pending_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(X(0), pose, prior_noise));
    pending_values_.insert(X(0), pose);
    isam2_->update(pending_graph_, pending_values_);
    pending_graph_.resize(0);
    pending_values_.clear();

    current_pose_idx_ = 0;
    current_pose_ = pose;
    last_solved_pose_ = pose;
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
    last_solved_pose_ = current_pose_;
    current_timestamp_ = 0.0;
    pending_odom_delta_ = gtsam::Pose2();
    pending_odom_steps_ = 0;
    pending_tags_.clear();

    resetHorizonWithPrior(current_pose_, prior_noise_);

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
    last_solved_pose_ = current_pose_;
    current_timestamp_ = 0.0;
    pending_odom_delta_ = gtsam::Pose2();
    pending_odom_steps_ = 0;
    pending_tags_.clear();
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

    if (config_.compact_odometry) {
        pending_odom_delta_ = pending_odom_delta_.compose(delta);
        pending_odom_steps_++;
        current_pose_ = last_solved_pose_.compose(pending_odom_delta_);
        current_timestamp_ = odom.timestamp;
    } else {
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
    }

    // Return current predicted estimate
    PoseEstimate estimate;
    estimate.x = current_pose_.x();
    estimate.y = current_pose_.y();
    estimate.theta = current_pose_.theta();
    estimate.timestamp = current_timestamp_;
    estimate.has_covariance = false;

    return estimate;
}

void PoseEstimator::addTagMeasurement(const TagMeasurement& measurement) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_tags_.push_back(measurement);
}

void PoseEstimator::addTagMeasurements(const std::vector<TagMeasurement>& measurements) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_tags_.insert(pending_tags_.end(), measurements.begin(), measurements.end());
}

PoseEstimate PoseEstimator::update() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        throw std::runtime_error("PoseEstimator not initialized");
    }

    bool has_vision = !pending_tags_.empty();
    if (!has_vision) {
        PoseEstimate estimate;
        estimate.x = current_pose_.x();
        estimate.y = current_pose_.y();
        estimate.theta = current_pose_.theta();
        estimate.timestamp = current_timestamp_;
        estimate.has_covariance = false;
        return estimate;
    }

    std::unordered_set<int32_t> visible_tags;

    if (config_.compact_odometry && pending_odom_steps_ > 0) {
        if (current_pose_idx_ + 1 >= kHorizonCapacity) {
            gtsam::SharedNoiseModel prior_noise = prior_noise_;
            try {
                gtsam::Values estimate = isam2_->calculateEstimate();
                gtsam::Marginals marginals(isam2_->getFactorsUnsafe(), estimate);
                gtsam::Matrix33 cov = marginals.marginalCovariance(X(current_pose_idx_));
                prior_noise = gtsam::noiseModel::Gaussian::Covariance(cov);
            } catch (...) {
                prior_noise = prior_noise_;
            }
            resetHorizonWithPrior(last_solved_pose_, prior_noise);
        }

        size_t prev_idx = current_pose_idx_;
        current_pose_idx_++;
        double scale = std::sqrt(static_cast<double>(pending_odom_steps_));
        auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(config_.odom_sigma_xy * scale,
                           config_.odom_sigma_xy * scale,
                           config_.odom_sigma_theta * scale));

        pending_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
            X(prev_idx), X(current_pose_idx_), pending_odom_delta_, odom_noise));

        gtsam::Pose2 predicted = last_solved_pose_.compose(pending_odom_delta_);
        pending_values_.insert(X(current_pose_idx_), predicted);
    }

    // Configuration objects
    gtsam::Cal3_S2 intrinsics(config_.fx, config_.fy, 0.0, config_.cx, config_.cy);
    gtsam::Pose3 extrinsics(
        gtsam::Rot3::Ypr(config_.camera_yaw, config_.camera_pitch, config_.camera_roll),
        gtsam::Point3(config_.camera_offset_x, config_.camera_offset_y, config_.camera_offset_z)
    );
    gtsam::Pose3 robot_pose(gtsam::Rot3::Ypr(current_pose_.theta(), 0.0, 0.0),
                            gtsam::Point3(current_pose_.x(), current_pose_.y(), 0.0));

    auto is_tag_visible = [&](const TagMeasurement& tag,
                              const gtsam::Pose3& tag_pose_world,
                              double tag_size) {
        gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(tag.turret_yaw_rad), gtsam::Point3(0, 0, 0));
        gtsam::Pose3 camera_pose = robot_pose.compose(turret_pose).compose(extrinsics);
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(camera_pose, intrinsics);

        double s = tag_size / 2.0;
        const std::array<gtsam::Point3, 4> corners_local = {
            gtsam::Point3(-s, -s, 0.0),
            gtsam::Point3( s, -s, 0.0),
            gtsam::Point3( s,  s, 0.0),
            gtsam::Point3(-s,  s, 0.0),
        };

        for (const auto& corner : corners_local) {
            auto projected = camera.projectSafe(tag_pose_world.transformFrom(corner));
            if (!projected.second) {
                return false;
            }
        }

        return true;
    };

    // Process pending tag measurements
    for (const auto& tag : pending_tags_) {
        auto landmark_opt = landmark_map_.getLandmark(tag.tag_id);
        if (!landmark_opt) {
            // Unknown tag ID - skip
            continue;
        }

        visible_tags.insert(tag.tag_id);

        // Create 8D noise model
        double sigma = (tag.pixel_sigma > 0) ? tag.pixel_sigma
                                             : config_.default_pixel_sigma;
        auto pixel_noise = gtsam::noiseModel::Isotropic::Sigma(8, sigma);

        // Convert Landmark to Pose3
        gtsam::Pose3 tag_pose_world(
            gtsam::Rot3::Ypr(landmark_opt->yaw, landmark_opt->pitch, landmark_opt->roll),
            gtsam::Point3(landmark_opt->x, landmark_opt->y, landmark_opt->z)
        );

        if (!is_tag_visible(tag, tag_pose_world, landmark_opt->size)) {
            continue;
        }

        // Add projection factor
        pending_graph_.add(TagProjectionFactor(
            X(current_pose_idx_),
            tag_pose_world,
            landmark_opt->size,
            intrinsics,
            extrinsics,
            tag.turret_yaw_rad,
            tag.corners,
            pixel_noise
        ));
    }
    pending_tags_.clear();

    auto solve_start = std::chrono::steady_clock::now();

    // Perform iSAM2 update
    if (pending_graph_.size() > 0 || pending_values_.size() > 0) {
        isam2_->update(pending_graph_, pending_values_);

        pending_graph_.resize(0);
        pending_values_.clear();
    }
    auto solve_end = std::chrono::steady_clock::now();
    last_solve_ms_ =
        std::chrono::duration<double, std::milli>(solve_end - solve_start).count();
    total_solve_ms_ += last_solve_ms_;
    solve_count_++;

    // Get updated estimate
    gtsam::Values estimate = isam2_->calculateEstimate();
    current_pose_ = estimate.at<gtsam::Pose2>(X(current_pose_idx_));
    last_solved_pose_ = current_pose_;
    pending_odom_delta_ = gtsam::Pose2();
    pending_odom_steps_ = 0;

    PoseEstimate result;
    result.x = current_pose_.x();
    result.y = current_pose_.y();
    result.theta = current_pose_.theta();
    result.timestamp = current_timestamp_;
    result.has_covariance = false;

#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        PoseEstimate vis = result;
        try {
            gtsam::Marginals marginals(isam2_->getFactorsUnsafe(), estimate);
            gtsam::Matrix33 cov = marginals.marginalCovariance(X(current_pose_idx_));
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    vis.covariance[static_cast<size_t>(i * 3 + j)] = cov(i, j);
                }
            }
            vis.has_covariance = true;
        } catch (...) {
            vis.has_covariance = false;
        }

        // Updated visualization call might be needed here, keeping basic pose log
        visualizer_->logPose(vis, current_pose_idx_);
    }
#endif

    return result;
}

void PoseEstimator::logTimeSeriesMetrics(int64_t step,
                                         const PoseEstimate& estimate,
                                         const PoseEstimate& true_pose,
                                         const PoseEstimate& odom_pose,
                                         const PoseEstimate& ekf_pose,
                                         double position_error,
                                         double odom_error,
                                         double ekf_error) {
    std::lock_guard<std::mutex> lock(mutex_);

#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        visualizer_->logTimeSeriesMetrics(step,
                                          estimate,
                                          true_pose,
                                          odom_pose,
                                          ekf_pose,
                                          position_error,
                                          odom_error,
                                          ekf_error);
    }
#else
    (void)step;
    (void)estimate;
    (void)true_pose;
    (void)odom_pose;
    (void)ekf_pose;
    (void)position_error;
    (void)odom_error;
    (void)ekf_error;
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

size_t PoseEstimator::getHorizonSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_pose_idx_ + 1;
}

bool PoseEstimator::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
}

double PoseEstimator::getLastSolveTimeMs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_solve_ms_;
}

double PoseEstimator::getAverageSolveTimeMs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return solve_count_ > 0 ? (total_solve_ms_ / static_cast<double>(solve_count_))
                            : 0.0;
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
