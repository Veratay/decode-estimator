#include "decode_estimator/tag_projection_factor.hpp"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>

namespace decode {

namespace {

constexpr double kCheiralityError = 1e3;

} // namespace

TagProjectionFactor::TagProjectionFactor(
    gtsam::Key poseKey,
    const gtsam::Pose3& tag_pose_world,
    double tag_size,
    const CameraModel& camera_intrinsics,
    const gtsam::Pose3& camera_extrinsics_body,
    double turret_yaw,
    const std::vector<std::pair<double, double>>& measured_corners,
    const gtsam::SharedNoiseModel& model)
    : Base(model, poseKey),
      tag_pose_world_(tag_pose_world),
      tag_size_(tag_size),
      camera_intrinsics_(camera_intrinsics),
      camera_extrinsics_body_(camera_extrinsics_body),
      turret_yaw_(turret_yaw) {
    
    if (measured_corners.size() != 4) {
        throw std::runtime_error("TagProjectionFactor requires 4 corners");
    }

    // Flatten measurements
    measured_.resize(8);
    for (size_t i = 0; i < 4; ++i) {
        measured_[2 * i] = measured_corners[i].first;
        measured_[2 * i + 1] = measured_corners[i].second;
    }

    // Precompute tag corners in local frame
    // Order: TL, TR, BR, BL (Standard AprilTag order)
    // Local frame: Center at (0,0,0), Z up (or out of tag), X right, Y down?
    // TagSLAM/AprilTag convention:
    // x right, y down, z forward (into tag? no, out).
    // Usually:
    // (-s/2, -s/2, 0) -> TL
    // ( s/2, -s/2, 0) -> TR
    // ( s/2,  s/2, 0) -> BR
    // (-s/2,  s/2, 0) -> BL
    // Note: This depends on the specific corner order of the detector.
    // Assuming standard order 0,1,2,3 correspond to TL, TR, BR, BL.
    
    double s = tag_size_ / 2.0;
    tag_corners_local_.emplace_back(-s, -s, 0.0); // TL
    tag_corners_local_.emplace_back( s, -s, 0.0); // TR
    tag_corners_local_.emplace_back( s,  s, 0.0); // BR
    tag_corners_local_.emplace_back(-s,  s, 0.0); // BL
}

gtsam::NonlinearFactor::shared_ptr TagProjectionFactor::clone() const {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new TagProjectionFactor(*this)));
}

void TagProjectionFactor::print(const std::string& s,
                                const gtsam::KeyFormatter& keyFormatter) const {
    std::cout << s << "TagProjectionFactor(" << keyFormatter(this->key()) << ")\n";
    std::cout << "  Measured: " << measured_.transpose() << "\n";
    tag_pose_world_.print("  Tag Pose World");
}

bool TagProjectionFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const {
    const TagProjectionFactor* e = dynamic_cast<const TagProjectionFactor*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           tag_pose_world_.equals(e->tag_pose_world_, tol) &&
           std::abs(tag_size_ - e->tag_size_) < tol &&
           std::abs(camera_intrinsics_.fx - e->camera_intrinsics_.fx) < tol &&
           std::abs(camera_intrinsics_.fy - e->camera_intrinsics_.fy) < tol &&
           std::abs(camera_intrinsics_.cx - e->camera_intrinsics_.cx) < tol &&
           std::abs(camera_intrinsics_.cy - e->camera_intrinsics_.cy) < tol &&
           std::abs(camera_intrinsics_.k1 - e->camera_intrinsics_.k1) < tol &&
           std::abs(camera_intrinsics_.k2 - e->camera_intrinsics_.k2) < tol &&
           std::abs(camera_intrinsics_.k3 - e->camera_intrinsics_.k3) < tol &&
           std::abs(camera_intrinsics_.p1 - e->camera_intrinsics_.p1) < tol &&
           std::abs(camera_intrinsics_.p2 - e->camera_intrinsics_.p2) < tol;
}

gtsam::Vector TagProjectionFactor::evaluateError(const gtsam::Pose2& pose,
                                                 gtsam::OptionalMatrixType H) const {
    auto compute_error = [&](const gtsam::Pose2& pose_value) -> gtsam::Vector {
        // 1. Lift Pose2 to Pose3
        gtsam::Pose3 robot_pose(gtsam::Rot3::Ypr(pose_value.theta(), 0.0, 0.0),
                                gtsam::Point3(pose_value.x(), pose_value.y(), 0.0));

        // 2. Compute Camera Pose
        // T_camera = T_robot * T_turret * T_ext
        // T_turret is a pure rotation around Z relative to robot
        gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(turret_yaw_), gtsam::Point3(0, 0, 0));
        gtsam::Pose3 fixed_offset = turret_pose.compose(camera_extrinsics_body_);
        gtsam::Pose3 camera_pose = robot_pose.compose(fixed_offset);

        // 3. Project points and accumulate error
        gtsam::Vector error(8);
        error.setZero();

        for (size_t i = 0; i < 4; ++i) {
            gtsam::Point3 world_point = tag_pose_world_.transformFrom(tag_corners_local_[i]);
            gtsam::Point2 projected;
            if (!projectPoint(camera_pose, camera_intrinsics_, world_point, &projected)) {
                return gtsam::Vector::Constant(8, kCheiralityError);
            }

            error(2 * i) = projected.x() - measured_(2 * i);
            error(2 * i + 1) = projected.y() - measured_(2 * i + 1);
        }

        return error;
    };

    gtsam::Vector error = compute_error(pose);

    if (H) {
        *H = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose2>(
            compute_error, pose, 1e-6);
    }

    return error;
}

} // namespace decode
