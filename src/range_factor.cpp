#include "decode_estimator/range_factor.hpp"

#include <gtsam/base/Vector.h>

#include <cassert>
#include <cmath>
#include <iostream>

namespace decode {

RangeToKnownLandmarkFactor::RangeToKnownLandmarkFactor(gtsam::Key poseKey,
                                                       const gtsam::Point2& landmark,
                                                       double measured,
                                                       const gtsam::SharedNoiseModel& model)
    : Base(model, poseKey), landmark_(landmark), measured_(measured) {
    assert(model->dim() == 1 && "Range noise model must be 1-dimensional");
}

gtsam::NonlinearFactor::shared_ptr RangeToKnownLandmarkFactor::clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        boost::make_shared<RangeToKnownLandmarkFactor>(*this));
}

void RangeToKnownLandmarkFactor::print(const std::string& s,
                                       const gtsam::KeyFormatter& keyFormatter) const {
    std::cout << s << "RangeToKnownLandmarkFactor(" << keyFormatter(this->key()) << ")\n";
    std::cout << "  landmark: [" << landmark_.x() << ", " << landmark_.y() << "]\n";
    std::cout << "  measured range: " << measured_ << " m\n";
    this->noiseModel_->print("  noise model: ");
}

bool RangeToKnownLandmarkFactor::equals(const gtsam::NonlinearFactor& expected,
                                        double tol) const {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           gtsam::traits<gtsam::Point2>::Equals(landmark_, e->landmark_, tol) &&
           std::abs(measured_ - e->measured_) <= tol;
}

gtsam::Vector RangeToKnownLandmarkFactor::evaluateError(
    const gtsam::Pose2& pose, boost::optional<gtsam::Matrix&> H) const {
    double dx = landmark_.x() - pose.x();
    double dy = landmark_.y() - pose.y();
    double range = std::sqrt(dx * dx + dy * dy);

    if (range < 1e-9) {
        range = 1e-9;
    }

    if (H) {
        double c = std::cos(pose.theta());
        double s = std::sin(pose.theta());
        double grad_x = dx / range;
        double grad_y = dy / range;

        *H = gtsam::Matrix::Zero(1, 3);
        (*H)(0, 0) = grad_x * c + grad_y * s;
        (*H)(0, 1) = -grad_x * s + grad_y * c;
        (*H)(0, 2) = 0.0;
    }

    double error = measured_ - range;
    return gtsam::Vector1(error);
}

} // namespace decode
