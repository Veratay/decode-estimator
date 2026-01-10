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
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double camera_offset_x = 0.0;
    double camera_offset_y = 0.0;
    double camera_offset_z = 0.5;
    double camera_roll = 0.0;
    double camera_pitch = 0.0;
    double camera_yaw = 0.0;

    // Tag gating
    bool enable_tag_gating = true;
    double min_tag_area_px = 50.0;
    double max_tag_view_angle_deg = 60.0;

    // Post-process estimates after vision gap resets
    bool enable_post_process = true;
    double post_process_vision_gap_s = 0.4;
    double post_process_settle_s = 1.0;
    int post_process_settle_updates = 3;

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

    PoseEstimate getPostProcessedEstimate() const;

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
    double last_tag_timestamp_ = -1.0;
    bool post_unstable_ = false;
    double unstable_until_timestamp_ = -1.0;
    int settle_updates_remaining_ = 0;
    Eigen::Vector3d last_stable_state_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d odom_since_stable_ = Eigen::Vector3d::Zero();

    mutable std::mutex mutex_;
};

} // namespace decode
