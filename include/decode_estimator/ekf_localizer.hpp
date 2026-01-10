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
    double default_pixel_sigma = 1.0;
    
    // Camera Parameters
    double fx = 1000.0;
    double fy = 1000.0;
    double cx = 320.0;
    double cy = 240.0;
    double camera_offset_x = 0.0;
    double camera_offset_y = 0.0;
    double camera_offset_z = 0.5;
    double camera_roll = 0.0;
    double camera_pitch = 0.0;
    double camera_yaw = 0.0;
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

    void addTagMeasurement(const TagMeasurement& measurement);

    PoseEstimate getCurrentEstimate() const;

    Eigen::Matrix3d getCovariance() const;

    bool isInitialized() const;

private:
    void updateTag(const TagMeasurement& tag);

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
