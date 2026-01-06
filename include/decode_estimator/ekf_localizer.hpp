#pragma once

#include "landmark_map.hpp"
#include "types.hpp"

#include <Eigen/Dense>

#include <cstdint>
#include <mutex>
#include <vector>

namespace decode {

struct EKFConfig {
    double prior_sigma_xy = 0.05;
    double prior_sigma_theta = 0.02;
    double odom_sigma_xy = 0.02;
    double odom_sigma_theta = 0.01;
    double default_bearing_sigma = 0.05;
    double default_distance_sigma = 0.2;
};

class EkfLocalizer {
public:
    explicit EkfLocalizer(const EKFConfig& config = EKFConfig());

    void initialize(const std::vector<Landmark>& landmarks,
                    double initial_x,
                    double initial_y,
                    double initial_theta);

    void reset();

    PoseEstimate processOdometry(const OdometryMeasurement& odom);

    void addBearingMeasurement(const BearingMeasurement& bearing);

    void addDistanceMeasurement(const DistanceMeasurement& distance);

    PoseEstimate getCurrentEstimate() const;

    Eigen::Matrix3d getCovariance() const;

    bool isInitialized() const;

private:
    void updateBearing(const BearingMeasurement& bearing);
    void updateDistance(const DistanceMeasurement& distance);

private:
    EKFConfig config_;
    bool initialized_ = false;
    LandmarkMap landmark_map_;
    Eigen::Vector3d state_;
    Eigen::Matrix3d covariance_;
    double current_timestamp_ = 0.0;

    mutable std::mutex mutex_;
};

} // namespace decode
