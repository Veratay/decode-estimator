#include "decode_estimator/ekf_localizer.hpp"

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

#include <cmath>
#include <stdexcept>

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

gtsam::Point2 rotatePoint(const gtsam::Point2& point, double angle_rad) {
    double c = std::cos(angle_rad);
    double s = std::sin(angle_rad);
    return gtsam::Point2(c * point.x() - s * point.y(),
                         s * point.x() + c * point.y());
}

gtsam::Pose2 cameraPoseFromRobot(const Eigen::Vector3d& state,
                                 const gtsam::Point2& camera_offset,
                                 double turret_yaw_rad) {
    gtsam::Pose2 robot_pose(state(0), state(1), state(2));
    gtsam::Point2 offset_in_robot = rotatePoint(camera_offset, turret_yaw_rad);
    gtsam::Pose2 turret_to_camera(offset_in_robot.x(), offset_in_robot.y(), turret_yaw_rad);
    return robot_pose.compose(turret_to_camera);
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

    state_(0) += c * odom.dx - s * odom.dy;
    state_(1) += s * odom.dx + c * odom.dy;
    state_(2) = wrapAngle(state_(2) + odom.dtheta);

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -s * odom.dx - c * odom.dy;
    F(1, 2) = c * odom.dx - s * odom.dy;

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
    Eigen::Matrix3d R = V * Q * V.transpose();

    covariance_ = F * covariance_ * F.transpose() + R;
    current_timestamp_ = odom.timestamp;

    PoseEstimate estimate;
    estimate.x = state_(0);
    estimate.y = state_(1);
    estimate.theta = state_(2);
    estimate.timestamp = current_timestamp_;
    estimate.has_covariance = false;
    return estimate;
}

void EkfLocalizer::addBearingMeasurement(const BearingMeasurement& bearing) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        throw std::runtime_error("EkfLocalizer not initialized");
    }
    updateBearing(bearing);
}

void EkfLocalizer::addDistanceMeasurement(const DistanceMeasurement& distance) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        throw std::runtime_error("EkfLocalizer not initialized");
    }
    updateDistance(distance);
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

void EkfLocalizer::updateBearing(const BearingMeasurement& bearing) {
    auto landmark_opt = landmark_map_.getLandmark(bearing.tag_id);
    if (!landmark_opt) {
        return;
    }

    gtsam::Point2 camera_offset(config_.camera_offset_x, config_.camera_offset_y);
    auto predicted_bearing = [&](const Eigen::Vector3d& state) {
        gtsam::Pose2 camera_pose = cameraPoseFromRobot(state, camera_offset,
                                                       bearing.turret_yaw_rad);
        double dx = landmark_opt->x() - camera_pose.x();
        double dy = landmark_opt->y() - camera_pose.y();
        return wrapAngle(std::atan2(dy, dx) - camera_pose.theta());
    };

    double predicted = predicted_bearing(state_);
    double residual = wrapAngle(bearing.bearing_rad - predicted);
    double sigma = (bearing.uncertainty_rad > 0.0) ? bearing.uncertainty_rad
                                                   : config_.default_bearing_sigma;

    const double eps = 1e-6;
    Eigen::RowVector3d H;
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d perturbed = state_;
        perturbed(i) += eps;
        double predicted_perturbed = predicted_bearing(perturbed);
        double diff = wrapAngle(predicted_perturbed - predicted);
        H(i) = diff / eps;
    }

    double S = (H * covariance_ * H.transpose())(0, 0) + sigma * sigma;
    if (S < 1e-9) {
        return;
    }
    Eigen::Vector3d K = covariance_ * H.transpose() / S;

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    covariance_ = (I - K * H) * covariance_ * (I - K * H).transpose()
                  + K * (sigma * sigma) * K.transpose();

    state_ += K * residual;
    state_(2) = wrapAngle(state_(2));
}

void EkfLocalizer::updateDistance(const DistanceMeasurement& distance) {
    auto landmark_opt = landmark_map_.getLandmark(distance.tag_id);
    if (!landmark_opt) {
        return;
    }

    if (distance.distance_m <= 0.0) {
        return;
    }

    gtsam::Point2 camera_offset(config_.camera_offset_x, config_.camera_offset_y);
    auto predicted_range = [&](const Eigen::Vector3d& state) {
        gtsam::Pose2 camera_pose = cameraPoseFromRobot(state, camera_offset,
                                                       distance.turret_yaw_rad);
        double dx = landmark_opt->x() - camera_pose.x();
        double dy = landmark_opt->y() - camera_pose.y();
        return std::sqrt(dx * dx + dy * dy);
    };

    double range = predicted_range(state_);
    if (range < 1e-6) {
        return;
    }

    double residual = distance.distance_m - range;
    double sigma = (distance.uncertainty_m > 0.0) ? distance.uncertainty_m
                                                  : config_.default_distance_sigma;

    const double eps = 1e-6;
    Eigen::RowVector3d H;
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d perturbed = state_;
        perturbed(i) += eps;
        double range_perturbed = predicted_range(perturbed);
        H(i) = (range_perturbed - range) / eps;
    }

    double S = (H * covariance_ * H.transpose())(0, 0) + sigma * sigma;
    if (S < 1e-9) {
        return;
    }
    Eigen::Vector3d K = covariance_ * H.transpose() / S;

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    covariance_ = (I - K * H) * covariance_ * (I - K * H).transpose()
                  + K * (sigma * sigma) * K.transpose();

    state_ += K * residual;
    state_(2) = wrapAngle(state_(2));
}

} // namespace decode
