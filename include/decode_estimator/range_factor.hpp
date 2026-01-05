#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace decode {

/**
 * @brief Unary factor for range measurement to a known landmark.
 *
 * This factor constrains a Pose2 based on the distance to a fixed landmark.
 * The measurement is the Euclidean range from the robot to the landmark
 * in the world frame.
 *
 * Error = measured_range - predicted_range
 */
class RangeToKnownLandmarkFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
public:
    using Base = gtsam::NoiseModelFactor1<gtsam::Pose2>;
    using This = RangeToKnownLandmarkFactor;
    using shared_ptr = boost::shared_ptr<This>;

    /// Default constructor for serialization
    RangeToKnownLandmarkFactor() = default;

    /**
     * @brief Constructor with range measurement
     * @param poseKey   Key for the robot pose variable
     * @param landmark  Known landmark position in world frame
     * @param measured  Measured range (meters)
     * @param model     Noise model (must be 1-dimensional for range)
     */
    RangeToKnownLandmarkFactor(gtsam::Key poseKey,
                               const gtsam::Point2& landmark,
                               double measured,
                               const gtsam::SharedNoiseModel& model);

    ~RangeToKnownLandmarkFactor() override = default;

    /// Clone for factor graph operations
    gtsam::NonlinearFactor::shared_ptr clone() const override;

    /// Print for debugging
    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

    /// Check equality
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;

    /**
     * @brief Evaluate error: measured - predicted range
     * @param pose  Current pose estimate
     * @param H     Optional Jacobian (1x3) with respect to pose
     * @return      1-dimensional error vector
     */
    gtsam::Vector evaluateError(
        const gtsam::Pose2& pose,
        boost::optional<gtsam::Matrix&> H = boost::none) const override;

    /// Get the known landmark position
    const gtsam::Point2& landmark() const { return landmark_; }

    /// Get the measured range
    double measured() const { return measured_; }

private:
    gtsam::Point2 landmark_; ///< Known landmark position in world frame
    double measured_ = 0.0;  ///< Measured range in meters
};

} // namespace decode
