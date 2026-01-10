#include "decode_estimator/bearing_factor.hpp"

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>

#include <cassert>
#include <iostream>

namespace decode {

namespace {

gtsam::Point2 rotatePoint(const gtsam::Point2& point, double angle_rad) {
    double c = std::cos(angle_rad);
    double s = std::sin(angle_rad);
    return gtsam::Point2(c * point.x() - s * point.y(),
                         s * point.x() + c * point.y());
}

gtsam::Pose2 cameraPoseFromRobot(const gtsam::Pose2& robot_pose,
                                 const gtsam::Point2& camera_offset,
                                 double turret_yaw_rad) {
    gtsam::Point2 offset_in_robot = rotatePoint(camera_offset, turret_yaw_rad);
    gtsam::Pose2 turret_to_camera(offset_in_robot.x(), offset_in_robot.y(), turret_yaw_rad);
    return robot_pose.compose(turret_to_camera);
}

} // namespace

BearingToKnownLandmarkFactor::BearingToKnownLandmarkFactor(gtsam::Key poseKey,
                                                           const gtsam::Point2& landmark,
                                                           const gtsam::Point2& camera_offset,
                                                           double turret_yaw_rad,
                                                           const gtsam::Rot2& measured,
                                                           const gtsam::SharedNoiseModel& model)
    : Base(model, poseKey),
      landmark_(landmark),
      measured_(measured),
      camera_offset_(camera_offset),
      turret_yaw_rad_(turret_yaw_rad) {
    // Verify noise model dimension
    assert(model->dim() == 1 && "Bearing noise model must be 1-dimensional");
}

BearingToKnownLandmarkFactor::BearingToKnownLandmarkFactor(gtsam::Key poseKey,
                                                           const gtsam::Point2& landmark,
                                                           const gtsam::Point2& camera_offset,
                                                           double turret_yaw_rad,
                                                           double bearing_rad,
                                                           const gtsam::SharedNoiseModel& model)
    : BearingToKnownLandmarkFactor(poseKey,
                                   landmark,
                                   camera_offset,
                                   turret_yaw_rad,
                                   gtsam::Rot2(bearing_rad),
                                   model) {}

BearingToKnownLandmarkFactor::BearingToKnownLandmarkFactor(gtsam::Key poseKey,
                                                           const gtsam::Point2& landmark,
                                                           const gtsam::Rot2& measured,
                                                           const gtsam::SharedNoiseModel& model)
    : BearingToKnownLandmarkFactor(poseKey,
                                   landmark,
                                   gtsam::Point2(0.0, 0.0),
                                   0.0,
                                   measured,
                                   model) {}

BearingToKnownLandmarkFactor::BearingToKnownLandmarkFactor(gtsam::Key poseKey,
                                                           const gtsam::Point2& landmark,
                                                           double bearing_rad,
                                                           const gtsam::SharedNoiseModel& model)
    : BearingToKnownLandmarkFactor(poseKey,
                                   landmark,
                                   gtsam::Point2(0.0, 0.0),
                                   0.0,
                                   gtsam::Rot2(bearing_rad),
                                   model) {}

gtsam::NonlinearFactor::shared_ptr BearingToKnownLandmarkFactor::clone() const {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        std::make_shared<BearingToKnownLandmarkFactor>(*this));
}

void BearingToKnownLandmarkFactor::print(const std::string& s,
                                          const gtsam::KeyFormatter& keyFormatter) const {
    std::cout << s << "BearingToKnownLandmarkFactor(" << keyFormatter(this->key()) << ")\n";
    std::cout << "  landmark: [" << landmark_.x() << ", " << landmark_.y() << "]\n";
    std::cout << "  measured bearing: " << measured_.theta() << " rad\n";
    std::cout << "  camera offset: [" << camera_offset_.x() << ", "
              << camera_offset_.y() << "]\n";
    std::cout << "  turret yaw: " << turret_yaw_rad_ << " rad\n";
    this->noiseModel_->print("  noise model: ");
}

bool BearingToKnownLandmarkFactor::equals(const gtsam::NonlinearFactor& expected,
                                           double tol) const {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           gtsam::traits<gtsam::Point2>::Equals(landmark_, e->landmark_, tol) &&
           gtsam::traits<gtsam::Rot2>::Equals(measured_, e->measured_, tol) &&
           gtsam::traits<gtsam::Point2>::Equals(camera_offset_, e->camera_offset_, tol) &&
           std::abs(turret_yaw_rad_ - e->turret_yaw_rad_) <= tol;
}

gtsam::Vector BearingToKnownLandmarkFactor::evaluateError(
    const gtsam::Pose2& pose, gtsam::OptionalMatrixType H) const {
    auto error_func = [this](const gtsam::Pose2& pose_in) {
        gtsam::Pose2 camera_pose = cameraPoseFromRobot(pose_in, camera_offset_,
                                                       turret_yaw_rad_);
        double dx = landmark_.x() - camera_pose.x();
        double dy = landmark_.y() - camera_pose.y();
        double c = std::cos(camera_pose.theta());
        double s = std::sin(camera_pose.theta());

        double local_x = c * dx + s * dy;
        double local_y = -s * dx + c * dy;
        double predicted_theta = std::atan2(local_y, local_x);

        double error = measured_.theta() - predicted_theta;
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;
        return gtsam::Vector1(error);
    };

    if (H) {
        *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose2>(error_func, pose);
    }

    return error_func(pose);
}

} // namespace decode
