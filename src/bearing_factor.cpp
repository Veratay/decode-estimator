#include "decode_estimator/bearing_factor.hpp"

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>

#include <cassert>
#include <iostream>

namespace decode {

BearingToKnownLandmarkFactor::BearingToKnownLandmarkFactor(gtsam::Key poseKey,
                                                           const gtsam::Point2& landmark,
                                                           const gtsam::Rot2& measured,
                                                           const gtsam::SharedNoiseModel& model)
    : Base(model, poseKey), landmark_(landmark), measured_(measured) {
    // Verify noise model dimension
    assert(model->dim() == 1 && "Bearing noise model must be 1-dimensional");
}

BearingToKnownLandmarkFactor::BearingToKnownLandmarkFactor(gtsam::Key poseKey,
                                                           const gtsam::Point2& landmark,
                                                           double bearing_rad,
                                                           const gtsam::SharedNoiseModel& model)
    : BearingToKnownLandmarkFactor(poseKey, landmark, gtsam::Rot2(bearing_rad), model) {}

gtsam::NonlinearFactor::shared_ptr BearingToKnownLandmarkFactor::clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        boost::make_shared<BearingToKnownLandmarkFactor>(*this));
}

void BearingToKnownLandmarkFactor::print(const std::string& s,
                                          const gtsam::KeyFormatter& keyFormatter) const {
    std::cout << s << "BearingToKnownLandmarkFactor(" << keyFormatter(this->key()) << ")\n";
    std::cout << "  landmark: [" << landmark_.x() << ", " << landmark_.y() << "]\n";
    std::cout << "  measured bearing: " << measured_.theta() << " rad\n";
    this->noiseModel_->print("  noise model: ");
}

bool BearingToKnownLandmarkFactor::equals(const gtsam::NonlinearFactor& expected,
                                           double tol) const {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           gtsam::traits<gtsam::Point2>::Equals(landmark_, e->landmark_, tol) &&
           gtsam::traits<gtsam::Rot2>::Equals(measured_, e->measured_, tol);
}

gtsam::Vector BearingToKnownLandmarkFactor::evaluateError(
    const gtsam::Pose2& pose, boost::optional<gtsam::Matrix&> H) const {

    // Compute bearing manually to avoid ABI issues with GTSAM's Pose2::bearing()
    // Transform landmark to robot frame: p_local = R^T * (p_world - t)
    double dx = landmark_.x() - pose.x();
    double dy = landmark_.y() - pose.y();
    double c = std::cos(pose.theta());
    double s = std::sin(pose.theta());

    // Rotate to robot frame
    double local_x = c * dx + s * dy;
    double local_y = -s * dx + c * dy;

    // Predicted bearing angle in robot frame
    double predicted_theta = std::atan2(local_y, local_x);

// Compute Jacobian if requested
    // For bearing error = measured - predicted
    // Need to compute how error changes with pose in body frame
    if (H) {
        double d2 = local_x * local_x + local_y * local_y;
        
        // Avoid division by zero
        if (d2 < 1e-9) {
            d2 = 1e-9;
        }

        *H = gtsam::Matrix::Zero(1, 3);
        // Jacobian in body frame using chain rule
        // Transform derivatives from local frame to body frame
        (*H)(0, 0) = -local_y / d2;  // d(error)/d(v_x)
        (*H)(0, 1) = local_x / d2;   // d(error)/d(v_y)
        (*H)(0, 2) = 1.0;            // d(error)/d(omega)
    }

    // Compute error on SO(2) manifold: measured - predicted
    // Use simple angle difference wrapped to [-pi, pi]
    double error = measured_.theta() - predicted_theta;

    // Normalize to [-pi, pi]
    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;

    // Return using GTSAM's Vector1 constructor
    return gtsam::Vector1(error);
}

} // namespace decode
