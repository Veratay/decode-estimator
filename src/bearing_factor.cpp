#include "decode_estimator/bearing_factor.hpp"

#include <gtsam/base/OptionalJacobian.h>
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

    // Compute predicted bearing from pose to landmark
    gtsam::Rot2 predicted = pose.bearing(landmark_, H);

    // Negate Jacobian since error = measured - predicted, but bearing returns d(predicted)/d(pose)
    if (H) {
        *H = -(*H);
    }

    // Compute error using traits::Local (SO(2) manifold)
    // Local(a, b) computes b - a on the manifold, so Local(predicted, measured) = measured - predicted
    return gtsam::traits<gtsam::Rot2>::Local(predicted, measured_);
}

} // namespace decode
