#include "decode_estimator/cheirality_factor.hpp"
#include <gtsam/base/numericalDerivative.h>

namespace decode {

CheiralityFactor::CheiralityFactor(gtsam::Key poseKey,
                                   const gtsam::Pose3& tag_pose_world,
                                   const gtsam::Pose3& camera_extrinsics_body,
                                   double turret_yaw,
                                   const gtsam::SharedNoiseModel& model,
                                   double min_z)
    : Base(model, poseKey),
      tag_pose_world_(tag_pose_world),
      camera_extrinsics_body_(camera_extrinsics_body),
      turret_yaw_(turret_yaw),
      min_z_(min_z) {}

gtsam::NonlinearFactor::shared_ptr CheiralityFactor::clone() const {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new CheiralityFactor(*this)));
}

void CheiralityFactor::print(const std::string& s,
                             const gtsam::KeyFormatter& keyFormatter) const {
    std::cout << s << "CheiralityFactor(" << keyFormatter(this->key()) << ")\n";
    std::cout << "  Min Z: " << min_z_ << "\n";
    tag_pose_world_.print("  Tag Pose World");
}

bool CheiralityFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const {
    const CheiralityFactor* e = dynamic_cast<const CheiralityFactor*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           tag_pose_world_.equals(e->tag_pose_world_, tol) &&
           std::abs(min_z_ - e->min_z_) < tol;
}

gtsam::Vector CheiralityFactor::evaluateError(const gtsam::Pose2& pose,
                                              gtsam::OptionalMatrixType H) const {
    auto compute_error = [&](const gtsam::Pose2& pose_value) -> gtsam::Vector {
        // 1. Robot Pose (3D)
        gtsam::Pose3 robot_pose(gtsam::Rot3::Ypr(pose_value.theta(), 0.0, 0.0),
                                gtsam::Point3(pose_value.x(), pose_value.y(), 0.0));

        // 2. Camera Pose
        gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(turret_yaw_), gtsam::Point3(0, 0, 0));
        gtsam::Pose3 camera_pose = robot_pose.compose(turret_pose).compose(camera_extrinsics_body_);

        // 3. Transform Tag to Camera Frame
        gtsam::Point3 tag_in_camera = camera_pose.transformTo(tag_pose_world_.translation());

        // 4. Check Z
        double z = tag_in_camera.z();
        gtsam::Vector error(1);
        if (z < min_z_) {
            error(0) = min_z_ - z; // Positive penalty
        } else {
            error(0) = 0.0;
        }
        return error;
    };

    if (H) {
        *H = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose2>(
            compute_error, pose, 1e-6);
    }

    return compute_error(pose);
}

} // namespace decode
