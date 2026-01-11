#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace decode {

/**
 * @brief Factor that penalizes tag centers being behind the camera.
 *
 * Implements a one-sided quadratic cost: error = max(0, min_z - z_camera).
 * This encourages the tag to be in front of the camera (positive Z).
 */
class CheiralityFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
public:
    using Base = gtsam::NoiseModelFactor1<gtsam::Pose2>;

    CheiralityFactor() = default;

    /**
     * @brief Constructor
     * @param poseKey Robot pose key
     * @param tag_pose_world Tag pose in world frame
     * @param camera_extrinsics_body Camera pose in robot body frame (T_body_camera)
     * @param turret_yaw Turret yaw relative to robot body
     * @param model Noise model (1-dim)
     * @param min_z Minimum Z distance in camera frame (default 0.1m)
     */
    CheiralityFactor(gtsam::Key poseKey,
                     const gtsam::Pose3& tag_pose_world,
                     const gtsam::Pose3& camera_extrinsics_body,
                     double turret_yaw,
                     const gtsam::SharedNoiseModel& model,
                     double min_z = 0.1);

    ~CheiralityFactor() override = default;

    gtsam::Vector evaluateError(const gtsam::Pose2& pose,
                                gtsam::OptionalMatrixType H = nullptr) const override;

    gtsam::NonlinearFactor::shared_ptr clone() const override;

    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;

private:
    gtsam::Pose3 tag_pose_world_;
    gtsam::Pose3 camera_extrinsics_body_;
    double turret_yaw_;
    double min_z_;
};

} // namespace decode
