#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "camera_model.hpp"

#include <vector>

namespace decode {

/**
 * @brief Factor that optimizes Robot Pose2 based on 4 detected tag corners.
 *
 * It projects the known 3D tag corners onto the camera image plane based on
 * the robot's Pose2 (lifted to 3D) and camera extrinsics.
 */
class TagProjectionFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
public:
    using Base = gtsam::NoiseModelFactor1<gtsam::Pose2>;
    using shared_ptr = std::shared_ptr<TagProjectionFactor>;

    /// Default constructor
    TagProjectionFactor() = default;

    /**
     * @brief Constructor
     * @param poseKey Robot pose key
     * @param tag_pose_world Tag pose in world frame
     * @param tag_size Tag side length (meters)
     * @param camera_intrinsics Camera intrinsics
     * @param camera_extrinsics_body Camera pose in robot body frame (T_body_camera)
     * @param turret_yaw Turret yaw relative to robot body
     * @param measured_corners Measured pixel coordinates (4 points: TL, TR, BR, BL)
     * @param model Noise model (isotropic, dim 8)
     */
    TagProjectionFactor(gtsam::Key poseKey,
                        const gtsam::Pose3& tag_pose_world,
                        double tag_size,
                        const CameraModel& camera_intrinsics,
                        const gtsam::Pose3& camera_extrinsics_body,
                        double turret_yaw,
                        const std::vector<std::pair<double, double>>& measured_corners,
                        const gtsam::SharedNoiseModel& model);

    ~TagProjectionFactor() override = default;

    /// Evaluate error (8-dim vector)
    gtsam::Vector evaluateError(const gtsam::Pose2& pose,
                                gtsam::OptionalMatrixType H = nullptr) const override;

    /// Clone
    gtsam::NonlinearFactor::shared_ptr clone() const override;

    /// Print
    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

    /// Equals
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;

private:
    gtsam::Pose3 tag_pose_world_;
    double tag_size_;
    CameraModel camera_intrinsics_;
    gtsam::Pose3 camera_extrinsics_body_; // T_body_camera
    double turret_yaw_;
    gtsam::Vector measured_; // 8 elements: u1, v1, u2, v2, ...

    // Cached 3D corners in Tag Frame
    std::vector<gtsam::Point3> tag_corners_local_;
};

} // namespace decode
