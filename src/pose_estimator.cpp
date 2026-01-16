#include "decode_estimator/pose_estimator.hpp"
#include "decode_estimator/tag_projection_factor.hpp"
#include "decode_estimator/cheirality_factor.hpp"
#include "decode_estimator/visualization.hpp"
#include "decode_estimator/camera_model.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <iostream>
#include <stdexcept>

#include <unistd.h>

// Use symbol shorthand for poses
using gtsam::symbol_shorthand::X;

namespace decode {

namespace {

MemoryUsage readMemoryUsage() {
    MemoryUsage usage;
    std::ifstream statm("/proc/self/statm");
    if (!statm.is_open()) {
        return usage;
    }

    uint64_t size = 0;
    uint64_t resident = 0;
    uint64_t shared = 0;
    uint64_t text = 0;
    uint64_t lib = 0;
    uint64_t data = 0;
    uint64_t dt = 0;
    if (!(statm >> size >> resident >> shared >> text >> lib >> data >> dt)) {
        return usage;
    }

    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0) {
        return usage;
    }

    usage.virtual_bytes = size * static_cast<uint64_t>(page_size);
    usage.resident_bytes = resident * static_cast<uint64_t>(page_size);
    usage.shared_bytes = shared * static_cast<uint64_t>(page_size);
    usage.data_bytes = data * static_cast<uint64_t>(page_size);
    usage.valid = true;
    return usage;
}

double quadArea(const std::vector<std::pair<double, double>>& corners) {
    if (corners.size() != 4) {
        return 0.0;
    }

    double area = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        const auto& p1 = corners[i];
        const auto& p2 = corners[(i + 1) % 4];
        area += p1.first * p2.second - p2.first * p1.second;
    }
    return std::abs(area) * 0.5;
}

double angleBetween(const gtsam::Point3& a, const gtsam::Point3& b) {
    double na = a.norm();
    double nb = b.norm();
    if (na <= 1e-9 || nb <= 1e-9) {
        return std::numeric_limits<double>::infinity();
    }
    double cos_angle = a.dot(b) / (na * nb);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    return std::acos(cos_angle);
}

} // namespace

PoseEstimator::PoseEstimator(const EstimatorConfig& config) : config_(config) {
    createNoiseModels();
    createISAM2();

#if DECODE_ENABLE_RERUN
    visualizer_ = std::make_unique<Visualizer>(config_.visualization_app_id);
    VisualizationConfig viz_config;
    viz_config.enabled = config_.enable_visualization;
    viz_config.app_id = config_.visualization_app_id;
    // Defaults for now, can be overridden by configureVisualization
    viz_config.stream_to_viewer = true; 
    visualizer_->configure(viz_config);
#endif
}

PoseEstimator::~PoseEstimator() = default;

void PoseEstimator::enableVisualization(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        visualizer_->setEnabled(enabled);
    }
#else
    (void)enabled;
#endif
}

void PoseEstimator::configureVisualization(bool stream, const std::string& url_or_path, const std::string& app_id) {
    std::lock_guard<std::mutex> lock(mutex_);
#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        VisualizationConfig config;
        config.enabled = visualizer_->isEnabled(); // Preserve enabled state
        config.stream_to_viewer = stream;
        config.app_id = app_id;
        if (stream) {
            config.stream_url = url_or_path;
        } else {
            config.save_path = url_or_path;
        }
        visualizer_->configure(config);
        
        // Re-log landmarks if initialized
        if (initialized_) {
            visualizer_->logLandmarks(landmark_map_);
        }
    }
#else
    (void)stream;
    (void)url_or_path;
    (void)app_id;
#endif
}

void PoseEstimator::flushVisualization() {
    std::lock_guard<std::mutex> lock(mutex_);
#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        visualizer_->flush();
    }
#endif
}

std::vector<std::pair<double,double>> PoseEstimator::getPredictedCorners(int32_t tag_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return {};
    
    auto landmark_opt = landmark_map_.getLandmark(tag_id);
    if (!landmark_opt) return {};
    
    gtsam::Pose3 camera_pose = getCurrentCameraPose();
    CameraModel intrinsics; // Use stored config
    intrinsics.fx = config_.fx; intrinsics.fy = config_.fy;
    intrinsics.cx = config_.cx; intrinsics.cy = config_.cy;
    // ... distortion params ...
    
    // Landmark pose
    gtsam::Pose3 tag_pose_world(
        gtsam::Rot3::Ypr(landmark_opt->yaw, landmark_opt->pitch, landmark_opt->roll),
        gtsam::Point3(landmark_opt->x, landmark_opt->y, landmark_opt->z)
    );
    
    double s = landmark_opt->size / 2.0;
    const std::array<gtsam::Point3, 4> corners_local = {
        gtsam::Point3(-s, -s, 0.0), gtsam::Point3( s, -s, 0.0),
        gtsam::Point3( s,  s, 0.0), gtsam::Point3(-s,  s, 0.0),
    };

    std::vector<std::pair<double,double>> projected;
    for (const auto& corner : corners_local) {
        gtsam::Point2 px;
        if (projectPoint(camera_pose, intrinsics, tag_pose_world.transformFrom(corner), &px)) {
            projected.push_back({px.x(), px.y()});
        } else {
             projected.push_back({-1, -1}); // Indicator of failure
        }
    }
    return projected;
}

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

gtsam::SharedNoiseModel PoseEstimator::computeTagNoiseModel(
    const TagMeasurement& tag,
    const gtsam::Pose2& robot_pose,
    const gtsam::Pose3& tag_pose_world,
    const gtsam::Pose3& camera_extrinsics,
    double turret_yaw_rad) {

    // Compute camera pose in world frame
    gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(turret_yaw_rad), gtsam::Point3(0, 0, 0));
    gtsam::Pose3 robot_pose_3d(gtsam::Rot3::Ypr(robot_pose.theta(), 0.0, 0.0),
                               gtsam::Point3(robot_pose.x(), robot_pose.y(), 0.0));
    gtsam::Pose3 camera_pose = robot_pose_3d.compose(turret_pose).compose(camera_extrinsics);

    // Calculate viewing angle
    gtsam::Point3 tag_normal_world = tag_pose_world.rotation().matrix() * gtsam::Point3(0, 0, 1);
    gtsam::Point3 tag_to_camera = camera_pose.translation() - tag_pose_world.translation();
    double viewing_angle = angleBetween(tag_normal_world, tag_to_camera);

    // Scale base sigma with viewing angle (quadratic model)
    double base_sigma = (tag.pixel_sigma > 0) ? tag.pixel_sigma : config_.default_pixel_sigma;
    double angle_factor = 1.0 + config_.pixel_sigma_angle_k * viewing_angle * viewing_angle;
    double adjusted_sigma = base_sigma * angle_factor;

    // Apply spatial correlation downweighting if enabled
    if (config_.enable_spatial_correlation) {
        double correlation_factor = computeSpatialCorrelationFactor(tag.tag_id, robot_pose);
        adjusted_sigma *= correlation_factor;
    }

    return gtsam::noiseModel::Isotropic::Sigma(8, adjusted_sigma);
}

double PoseEstimator::computeSpatialCorrelationFactor(
    int32_t tag_id,
    const gtsam::Pose2& current_pose) {

    if (!config_.enable_spatial_correlation) {
        return 1.0;
    }

    // Check measurement history for nearby measurements of same tag
    for (const auto& record : measurement_history_) {
        if (record.tag_id != tag_id) {
            continue;
        }

        double distance = current_pose.range(record.robot_pose);
        if (distance < config_.correlation_distance_m) {
            return config_.correlation_downweight_factor;
        }
    }

    return 1.0;  // No correlation
}

std::vector<std::pair<double, double>> PoseEstimator::correctMeasurementBias(
    const TagMeasurement& tag,
    const gtsam::Pose2& robot_pose,
    const gtsam::Pose3& tag_pose_world,
    const gtsam::Pose3& camera_extrinsics,
    double turret_yaw_rad) {

    if (!config_.enable_bias_correction) {
        return tag.corners;  // No correction
    }

    // Compute camera pose and viewing angle
    gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(turret_yaw_rad), gtsam::Point3(0, 0, 0));
    gtsam::Pose3 robot_pose_3d(gtsam::Rot3::Ypr(robot_pose.theta(), 0.0, 0.0),
                               gtsam::Point3(robot_pose.x(), robot_pose.y(), 0.0));
    gtsam::Pose3 camera_pose = robot_pose_3d.compose(turret_pose).compose(camera_extrinsics);

    gtsam::Point3 tag_normal = tag_pose_world.rotation().matrix() * gtsam::Point3(0, 0, 1);
    gtsam::Point3 tag_to_camera = camera_pose.translation() - tag_pose_world.translation();
    double viewing_angle = angleBetween(tag_normal, tag_to_camera);

    // Compute bias magnitude (increases with angle)
    double radial_bias = config_.radial_bias_k * viewing_angle * viewing_angle;

    // Create camera intrinsics from config
    CameraModel intrinsics;
    intrinsics.fx = config_.fx;
    intrinsics.fy = config_.fy;
    intrinsics.cx = config_.cx;
    intrinsics.cy = config_.cy;
    intrinsics.k1 = config_.k1;
    intrinsics.k2 = config_.k2;
    intrinsics.k3 = config_.k3;
    intrinsics.p1 = config_.p1;
    intrinsics.p2 = config_.p2;

    // Compute tag center in image
    gtsam::Point3 tag_center_world = tag_pose_world.translation();
    gtsam::Point2 tag_center_uv;
    if (!projectPoint(camera_pose, intrinsics, tag_center_world, &tag_center_uv)) {
        // If projection fails, return uncorrected corners
        return tag.corners;
    }

    // Apply radial bias correction to each corner
    std::vector<std::pair<double, double>> corrected_corners;
    corrected_corners.reserve(tag.corners.size());

    for (const auto& corner : tag.corners) {
        double du = corner.first - tag_center_uv.x();
        double dv = corner.second - tag_center_uv.y();
        double r = std::sqrt(du*du + dv*dv);

        if (r > 1e-6) {  // Avoid division by zero
            // Apply bias correction outward from center
            double scale = 1.0 + (radial_bias / r);
            corrected_corners.emplace_back(
                tag_center_uv.x() + du * scale,
                tag_center_uv.y() + dv * scale
            );
        } else {
            // Corner is at center, no correction needed
            corrected_corners.push_back(corner);
        }
    }

    return corrected_corners;
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
    last_tag_timestamp_ = -1.0;
    post_unstable_ = false;
    unstable_until_timestamp_ = -1.0;
    settle_updates_remaining_ = 0;
    last_stable_pose_ = current_pose_;
    odom_since_stable_ = gtsam::Pose2();

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
    last_tag_timestamp_ = -1.0;
    post_unstable_ = false;
    unstable_until_timestamp_ = -1.0;
    settle_updates_remaining_ = 0;
    last_stable_pose_ = gtsam::Pose2();
    odom_since_stable_ = gtsam::Pose2();

    createISAM2();
}

PoseEstimate PoseEstimator::processOdometry(const OdometryMeasurement& odom) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        throw std::runtime_error("PoseEstimator not initialized");
    }

    // Create relative pose from odometry (in robot body frame)
    gtsam::Pose2 delta(odom.dx, odom.dy, odom.dtheta);

#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        visualizer_->logOdometryDelta(delta, odom.timestamp);
    }
#endif

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

    if (config_.enable_post_process) {
        if (post_unstable_) {
            odom_since_stable_ = odom_since_stable_.compose(delta);
        } else {
            last_stable_pose_ = current_pose_;
            odom_since_stable_ = gtsam::Pose2();
        }
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
    last_turret_yaw_rad_ = measurement.turret_yaw_rad;
}

void PoseEstimator::addTagMeasurements(const std::vector<TagMeasurement>& measurements) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_tags_.insert(pending_tags_.end(), measurements.begin(), measurements.end());
    if (!measurements.empty()) {
        last_turret_yaw_rad_ = measurements.back().turret_yaw_rad;
    }
}

PoseEstimate PoseEstimator::update() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        throw std::runtime_error("PoseEstimator not initialized");
    }

    // Configuration objects
    CameraModel intrinsics;
    intrinsics.fx = config_.fx;
    intrinsics.fy = config_.fy;
    intrinsics.cx = config_.cx;
    intrinsics.cy = config_.cy;
    intrinsics.k1 = config_.k1;
    intrinsics.k2 = config_.k2;
    intrinsics.k3 = config_.k3;
    intrinsics.p1 = config_.p1;
    intrinsics.p2 = config_.p2;
    gtsam::Pose3 extrinsics(
        gtsam::Rot3::Ypr(config_.camera_yaw, config_.camera_pitch, config_.camera_roll),
        gtsam::Point3(config_.camera_offset_x, config_.camera_offset_y, config_.camera_offset_z)
    );
    gtsam::Pose3 robot_pose(gtsam::Rot3::Ypr(current_pose_.theta(), 0.0, 0.0),
                            gtsam::Point3(current_pose_.x(), current_pose_.y(), 0.0));

    bool has_vision = !pending_tags_.empty();
    if (!has_vision) {
        PoseEstimate estimate;
        estimate.x = current_pose_.x();
        estimate.y = current_pose_.y();
        estimate.theta = current_pose_.theta();
        estimate.timestamp = current_timestamp_;
        estimate.has_covariance = false;
#if DECODE_ENABLE_RERUN
        if (visualizer_) {
            gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(last_turret_yaw_rad_),
                                     gtsam::Point3(0, 0, 0));
            gtsam::Pose3 camera_pose = robot_pose.compose(turret_pose).compose(extrinsics);
            visualizer_->logCamera(camera_pose, intrinsics);
            visualizer_->logPose(estimate, current_pose_idx_);
        }
#endif
        recordMemoryUsageLocked();
        return estimate;
    }

    if (config_.compact_odometry && pending_odom_steps_ > 0) {
        if (current_pose_idx_ + 1 >= kHorizonCapacity) {
            auto reset_start = std::chrono::steady_clock::now();
            gtsam::SharedNoiseModel prior_noise = prior_noise_;
            try {
                auto cov_start = std::chrono::steady_clock::now();
                gtsam::Values estimate = isam2_->calculateEstimate();
                gtsam::Marginals marginals(isam2_->getFactorsUnsafe(), estimate);
                gtsam::Matrix33 cov = marginals.marginalCovariance(X(current_pose_idx_));
                prior_noise = gtsam::noiseModel::Gaussian::Covariance(cov);
                auto cov_end = std::chrono::steady_clock::now();
                last_horizon_cov_ms_ =
                    std::chrono::duration<double, std::milli>(cov_end - cov_start).count();
            } catch (...) {
                prior_noise = prior_noise_;
                last_horizon_cov_ms_ = 0.0;
            }
            resetHorizonWithPrior(last_solved_pose_, prior_noise);
            auto reset_end = std::chrono::steady_clock::now();
            last_horizon_reset_ms_ =
                std::chrono::duration<double, std::milli>(reset_end - reset_start).count();
            last_horizon_reset_pose_index_ = current_pose_idx_;
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

    auto is_tag_visible = [&](const TagMeasurement& tag,
                              const gtsam::Pose3& tag_pose_world,
                              double tag_size) {
        if (config_.enable_tag_gating) {
            double area_px = quadArea(tag.corners);
            if (area_px < config_.min_tag_area_px) {
                return false;
            }
        }

        gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(tag.turret_yaw_rad), gtsam::Point3(0, 0, 0));
        gtsam::Pose3 camera_pose = robot_pose.compose(turret_pose).compose(extrinsics);

        double s = tag_size / 2.0;
        const std::array<gtsam::Point3, 4> corners_local = {
            gtsam::Point3(-s, -s, 0.0),
            gtsam::Point3( s, -s, 0.0),
            gtsam::Point3( s,  s, 0.0),
            gtsam::Point3(-s,  s, 0.0),
        };

        for (const auto& corner : corners_local) {
            gtsam::Point2 projected;
            if (!projectPoint(camera_pose, intrinsics,
                              tag_pose_world.transformFrom(corner),
                              &projected)) {
                return false;
            }
        }

        if (config_.enable_tag_gating) {
            gtsam::Point3 tag_normal_world =
                tag_pose_world.rotation().matrix() * gtsam::Point3(0.0, 0.0, 1.0);
            gtsam::Point3 tag_to_camera =
                camera_pose.translation() - tag_pose_world.translation();
            double angle = angleBetween(tag_normal_world, tag_to_camera);
            double max_angle = config_.max_tag_view_angle_deg * M_PI / 180.0;
            if (angle > max_angle) {
                return false;
            }
        }

        return true;
    };

    double max_tag_timestamp = -std::numeric_limits<double>::infinity();
    for (const auto& tag : pending_tags_) {
        max_tag_timestamp = std::max(max_tag_timestamp, tag.timestamp);
    }
    if (config_.enable_post_process) {
        if (last_tag_timestamp_ >= 0.0 &&
            (max_tag_timestamp - last_tag_timestamp_) > config_.post_process_vision_gap_s) {
            post_unstable_ = true;
            unstable_until_timestamp_ = max_tag_timestamp + config_.post_process_settle_s;
            settle_updates_remaining_ = config_.post_process_settle_updates;
            last_stable_pose_ = current_pose_;
            odom_since_stable_ = gtsam::Pose2();
        }
        last_tag_timestamp_ = max_tag_timestamp;
    }
    // Process pending tag measurements
    for (const auto& tag : pending_tags_) {
            auto landmark_opt = landmark_map_.getLandmark(tag.tag_id);
            if (!landmark_opt) {
                // Unknown tag ID - skip
                continue;
            }

            gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(tag.turret_yaw_rad),
                                     gtsam::Point3(0, 0, 0));
            gtsam::Pose3 camera_pose = robot_pose.compose(turret_pose).compose(extrinsics);

            // Convert Landmark to Pose3
            gtsam::Pose3 tag_pose_world(
                gtsam::Rot3::Ypr(landmark_opt->yaw, landmark_opt->pitch, landmark_opt->roll),
                gtsam::Point3(landmark_opt->x, landmark_opt->y, landmark_opt->z)
            );

#if DECODE_ENABLE_RERUN
            if (visualizer_ && visualizer_->isEnabled()) {
                double s = landmark_opt->size / 2.0;
                const std::array<gtsam::Point3, 4> corners_local = {
                    gtsam::Point3(-s, -s, 0.0),
                    gtsam::Point3( s, -s, 0.0),
                    gtsam::Point3( s,  s, 0.0),
                    gtsam::Point3(-s,  s, 0.0),
                };

                std::vector<gtsam::Point3> corners_world;
                corners_world.reserve(corners_local.size());
                std::vector<std::pair<double,double>> predicted_px;

                for (const auto& corner : corners_local) {
                    gtsam::Point3 world_pt = tag_pose_world.transformFrom(corner);
                    corners_world.push_back(world_pt);
                    
                    gtsam::Point2 px;
                    if (projectPoint(camera_pose, intrinsics, world_pt, &px)) {
                        predicted_px.push_back({px.x(), px.y()});
                    } else {
                        predicted_px.push_back({0,0}); 
                    }
                }

                visualizer_->logCamera(camera_pose, intrinsics);
                visualizer_->logTagCornerRays(tag.tag_id, camera_pose, corners_world);
                
                // Log comparison
                visualizer_->logTagCornerComparison(tag.tag_id, tag.corners, predicted_px);
                
                // Log frames
                visualizer_->logCoordinateFrames(current_pose_, tag.turret_yaw_rad, extrinsics);
            }
#endif

            if (!is_tag_visible(tag, tag_pose_world, landmark_opt->size)) {
                continue;
            }

            // Apply bias correction to corners
            auto corrected_corners = correctMeasurementBias(
                tag, current_pose_, tag_pose_world, extrinsics, tag.turret_yaw_rad);

            // Create viewing angle-dependent noise model with spatial correlation
            gtsam::SharedNoiseModel pixel_noise = computeTagNoiseModel(
                tag, current_pose_, tag_pose_world, extrinsics, tag.turret_yaw_rad);

            // Apply robust loss if enabled
            if (config_.enable_robust_tag_loss) {
                gtsam::noiseModel::mEstimator::Base::shared_ptr estimator;
                switch (config_.robust_tag_loss) {
                    case RobustLossType::Huber:
                        estimator = gtsam::noiseModel::mEstimator::Huber::Create(
                            config_.robust_tag_loss_k);
                        break;
                    case RobustLossType::Tukey:
                        estimator = gtsam::noiseModel::mEstimator::Tukey::Create(
                            config_.robust_tag_loss_k);
                        break;
                    case RobustLossType::Cauchy:
                        estimator = gtsam::noiseModel::mEstimator::Cauchy::Create(
                            config_.robust_tag_loss_k);
                        break;
                }
                pixel_noise = gtsam::noiseModel::Robust::Create(estimator, pixel_noise);
            }

            // Add projection factor
            pending_graph_.add(TagProjectionFactor(
                X(current_pose_idx_),
                tag_pose_world,
                landmark_opt->size,
                intrinsics,
                extrinsics,
                tag.turret_yaw_rad,
                corrected_corners,
                pixel_noise
            ));

            if (config_.enable_cheirality_check) {
                auto cheirality_noise = gtsam::noiseModel::Isotropic::Sigma(1, config_.cheirality_sigma);
                pending_graph_.add(CheiralityFactor(
                    X(current_pose_idx_),
                    tag_pose_world,
                    extrinsics,
                    tag.turret_yaw_rad,
                    cheirality_noise,
                    config_.min_tag_z_distance
                ));
            }

            // Record measurement for spatial correlation tracking
            if (config_.enable_spatial_correlation) {
                measurement_history_.push_back({
                    tag.tag_id,
                    current_pose_idx_,
                    current_pose_,
                    tag.timestamp
                });

                // Maintain sliding window
                if (measurement_history_.size() > config_.correlation_history_size) {
                    measurement_history_.pop_front();
                }
            }

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
    if (config_.enable_post_process) {
        if (post_unstable_) {
            if (settle_updates_remaining_ > 0) {
                settle_updates_remaining_--;
            }
        }
        if (post_unstable_ &&
            max_tag_timestamp >= unstable_until_timestamp_ &&
            settle_updates_remaining_ <= 0) {
            post_unstable_ = false;
            last_stable_pose_ = current_pose_;
            odom_since_stable_ = gtsam::Pose2();
        }
        if (!post_unstable_) {
            last_stable_pose_ = current_pose_;
            odom_since_stable_ = gtsam::Pose2();
        }
    }

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

    recordMemoryUsageLocked();
    return result;
}

void PoseEstimator::logTimeSeriesMetrics(int64_t step,
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
                                         double post_ekf_error) {
    std::lock_guard<std::mutex> lock(mutex_);

#if DECODE_ENABLE_RERUN
    if (visualizer_) {
        visualizer_->logTimeSeriesMetrics(step,
                                          estimate,
                                          true_pose,
                                          odom_pose,
                                          ekf_pose,
                                          post_estimate,
                                          post_ekf,
                                          position_error,
                                          odom_error,
                                          ekf_error,
                                          post_position_error,
                                          post_ekf_error);
    }
#else
    (void)step;
    (void)estimate;
    (void)true_pose;
    (void)odom_pose;
    (void)ekf_pose;
    (void)post_estimate;
    (void)post_ekf;
    (void)position_error;
    (void)odom_error;
    (void)ekf_error;
    (void)post_position_error;
    (void)post_ekf_error;
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

PoseEstimate PoseEstimator::getPostProcessedEstimate() const {
    std::lock_guard<std::mutex> lock(mutex_);

    gtsam::Pose2 pose = current_pose_;
    if (config_.enable_post_process && post_unstable_) {
        pose = last_stable_pose_.compose(odom_since_stable_);
    }

    PoseEstimate result;
    result.x = pose.x();
    result.y = pose.y();
    result.theta = pose.theta();
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

MemoryUsage PoseEstimator::getLastMemoryUsage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_memory_usage_;
}

DiagnosticsSnapshot PoseEstimator::getDiagnosticsSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    DiagnosticsSnapshot snapshot;
    snapshot.pending_tags = pending_tags_.size();
    snapshot.pending_graph_factors = pending_graph_.size();
    snapshot.pending_values = pending_values_.size();
    snapshot.current_pose_index = current_pose_idx_;
    snapshot.horizon_capacity = kHorizonCapacity;
    snapshot.last_solve_ms = last_solve_ms_;
    snapshot.avg_solve_ms = solve_count_ > 0
        ? (total_solve_ms_ / static_cast<double>(solve_count_))
        : 0.0;
    snapshot.last_horizon_reset_ms = last_horizon_reset_ms_;
    snapshot.last_horizon_cov_ms = last_horizon_cov_ms_;
    snapshot.last_horizon_reset_pose_index = last_horizon_reset_pose_index_;
    return snapshot;
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

void PoseEstimator::recordMemoryUsageLocked() {
    last_memory_usage_ = readMemoryUsage();
}

gtsam::Pose3 PoseEstimator::getCurrentCameraPose() const {
    // 1. Lift current 2D robot pose to 3D
    gtsam::Pose3 robot_pose(
        gtsam::Rot3::Ypr(current_pose_.theta(), 0.0, 0.0),
        gtsam::Point3(current_pose_.x(), current_pose_.y(), 0.0)
    );

    // 2. Apply turret rotation
    gtsam::Pose3 turret_pose(
        gtsam::Rot3::Yaw(last_turret_yaw_rad_),
        gtsam::Point3(0, 0, 0)
    );

    // 3. Apply camera extrinsics (body to camera)
    gtsam::Pose3 extrinsics(
        gtsam::Rot3::Ypr(config_.camera_yaw, config_.camera_pitch, config_.camera_roll),
        gtsam::Point3(config_.camera_offset_x, config_.camera_offset_y, config_.camera_offset_z)
    );

    // 4. Compose full transformation: World -> Robot -> Turret -> Camera
    return robot_pose.compose(turret_pose).compose(extrinsics);
}

} // namespace decode
