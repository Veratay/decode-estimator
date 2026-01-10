#include "decode_estimator/ekf_localizer.hpp"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

#include <cmath>
#include <stdexcept>
#include <iostream>

namespace decode {

namespace {

double wrapAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

} // namespace

EkfLocalizer::EkfLocalizer(const EKFConfig& config) : config_(config) {}

void EkfLocalizer::initialize(const std::vector<Landmark>& landmarks,
                              double initial_x,
                              double initial_y,
                              double initial_theta) {
    std::lock_guard<std::mutex> lock(mutex_);

    landmark_map_.loadFromVector(landmarks);
    state_ << initial_x, initial_y, initial_theta;
    covariance_.setZero();
    covariance_(0, 0) = config_.prior_sigma_xy * config_.prior_sigma_xy;
    covariance_(1, 1) = config_.prior_sigma_xy * config_.prior_sigma_xy;
    covariance_(2, 2) = config_.prior_sigma_theta * config_.prior_sigma_theta;
    current_timestamp_ = 0.0;
    initialized_ = true;
}

void EkfLocalizer::reset() {
    std::lock_guard<std::mutex> lock(mutex_);

    initialized_ = false;
    state_.setZero();
    covariance_.setZero();
    current_timestamp_ = 0.0;
    landmark_map_.clear();
}

PoseEstimate EkfLocalizer::processOdometry(const OdometryMeasurement& odom) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        throw std::runtime_error("EkfLocalizer not initialized");
    }

    double theta = state_(2);
    double c = std::cos(theta);
    double s = std::sin(theta);

    // Predict state
    state_(0) += c * odom.dx - s * odom.dy;
    state_(1) += s * odom.dx + c * odom.dy;
    state_(2) = wrapAngle(state_(2) + odom.dtheta);

    // Jacobian of process model F w.r.t state
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -s * odom.dx - c * odom.dy;
    F(1, 2) = c * odom.dx - s * odom.dy;

    // Jacobian of process model V w.r.t noise
    Eigen::Matrix3d V = Eigen::Matrix3d::Zero();
    V(0, 0) = c;
    V(0, 1) = -s;
    V(1, 0) = s;
    V(1, 1) = c;
    V(2, 2) = 1.0;

    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q(0, 0) = config_.odom_sigma_xy * config_.odom_sigma_xy;
    Q(1, 1) = config_.odom_sigma_xy * config_.odom_sigma_xy;
    Q(2, 2) = config_.odom_sigma_theta * config_.odom_sigma_theta;
    
    // Propagate covariance
    covariance_ = F * covariance_ * F.transpose() + V * Q * V.transpose();
    current_timestamp_ = odom.timestamp;

    PoseEstimate estimate;
    estimate.x = state_(0);
    estimate.y = state_(1);
    estimate.theta = state_(2);
    estimate.timestamp = current_timestamp_;
    estimate.has_covariance = false;
    return estimate;
}

void EkfLocalizer::addTagMeasurement(const TagMeasurement& measurement) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        throw std::runtime_error("EkfLocalizer not initialized");
    }
    updateTag(measurement);
}

PoseEstimate EkfLocalizer::getCurrentEstimate() const {
    std::lock_guard<std::mutex> lock(mutex_);

    PoseEstimate estimate;
    estimate.x = state_(0);
    estimate.y = state_(1);
    estimate.theta = state_(2);
    estimate.timestamp = current_timestamp_;
    estimate.has_covariance = false;
    return estimate;
}

Eigen::Matrix3d EkfLocalizer::getCovariance() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return covariance_;
}

bool EkfLocalizer::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
}

void EkfLocalizer::updateTag(const TagMeasurement& tag) {
    auto landmark_opt = landmark_map_.getLandmark(tag.tag_id);
    if (!landmark_opt) {
        return;
    }
    
    if (tag.corners.size() != 4) return;

    // Measurement vector z (8x1)
    Eigen::Matrix<double, 8, 1> z;
    for(int i=0; i<4; ++i) {
        z(2*i) = tag.corners[i].first;
        z(2*i+1) = tag.corners[i].second;
    }

    // Camera Intrinsics
    gtsam::Cal3_S2 intrinsics(config_.fx, config_.fy, 0, config_.cx, config_.cy);
    
    // Extrinsics (constant)
    gtsam::Pose3 extrinsics(
        gtsam::Rot3::Ypr(config_.camera_yaw, config_.camera_pitch, config_.camera_roll),
        gtsam::Point3(config_.camera_offset_x, config_.camera_offset_y, config_.camera_offset_z)
    );

    // Tag World Pose
    gtsam::Pose3 tag_pose(
        gtsam::Rot3::Ypr(landmark_opt->yaw, landmark_opt->pitch, landmark_opt->roll),
        gtsam::Point3(landmark_opt->x, landmark_opt->y, landmark_opt->z)
    );

    double s = landmark_opt->size / 2.0;
    std::vector<gtsam::Point3> corners_local = {
        {-s, -s, 0}, {s, -s, 0}, {s, s, 0}, {-s, s, 0}
    };

    // Measurement function h(x) and Jacobian H (8x3)
    auto predict = [&](const Eigen::Vector3d& x_state) -> Eigen::Matrix<double, 8, 1> {
        // Robot Pose3
        gtsam::Pose3 robot_pose(gtsam::Rot3::Ypr(x_state(2), 0, 0),
                                gtsam::Point3(x_state(0), x_state(1), 0));
        
        // Turret rotation (assumed 0 or handled externally - here using tag.turret_yaw_rad if applicable)
        // But the TagMeasurement struct has turret_yaw_rad.
        gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(tag.turret_yaw_rad), gtsam::Point3(0,0,0));
        
        gtsam::Pose3 camera_pose = robot_pose.compose(turret_pose).compose(extrinsics);
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera(camera_pose, intrinsics);

        Eigen::Matrix<double, 8, 1> h_vec;
        for(int i=0; i<4; ++i) {
            gtsam::Point3 pt_world = tag_pose.transformFrom(corners_local[i]);
            try {
                gtsam::Point2 uv = camera.project(pt_world);
                h_vec(2*i) = uv.x();
                h_vec(2*i+1) = uv.y();
            } catch (...) {
                h_vec(2*i) = 0;
                h_vec(2*i+1) = 0;
            }
        }
        return h_vec;
    };

    Eigen::Matrix<double, 8, 1> z_pred = predict(state_);
    Eigen::Matrix<double, 8, 1> residual = z - z_pred;

    // Compute H by numerical differentiation
    Eigen::Matrix<double, 8, 3> H;
    double eps = 1e-5;
    for(int i=0; i<3; ++i) {
        Eigen::Vector3d state_plus = state_;
        state_plus(i) += eps;
        Eigen::Matrix<double, 8, 1> z_plus = predict(state_plus);
        H.col(i) = (z_plus - z_pred) / eps;
    }

    // Measurement noise R
    double sigma = (tag.pixel_sigma > 0) ? tag.pixel_sigma : config_.default_pixel_sigma;
    Eigen::Matrix<double, 8, 8> R = Eigen::Matrix<double, 8, 8>::Identity() * (sigma * sigma);

    // EKF Update
    // S = H*P*H' + R
    Eigen::Matrix<double, 8, 8> S = H * covariance_ * H.transpose() + R;
    
    // K = P*H'*S^-1
    Eigen::Matrix<double, 3, 8> K = covariance_ * H.transpose() * S.inverse();

    // Update state
    Eigen::Vector3d correction = K * residual;
    state_ += correction;
    state_(2) = wrapAngle(state_(2));

    // Update covariance
    // P = (I - K*H)*P
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    covariance_ = (I - K * H) * covariance_;
}

} // namespace decode