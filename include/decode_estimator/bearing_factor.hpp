#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>

namespace decode {

/**
 * @brief Unary factor for bearing measurement to a known landmark.
 *
 * This factor constrains a Pose2 based on the bearing angle to a landmark
 * with known position. Unlike GTSAM's BearingFactor which treats the landmark
 * as a variable, this factor uses a fixed landmark position for efficiency.
 *
 * The measurement is the bearing angle from the robot to the landmark in
 * the robot's local frame (0 = forward, positive = counterclockwise).
 *
 * Error = measured_bearing - predicted_bearing (on SO(2) manifold)
 */
class BearingToKnownLandmarkFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
public:
    using Base = gtsam::NoiseModelFactor1<gtsam::Pose2>;
    using This = BearingToKnownLandmarkFactor;
    using shared_ptr = std::shared_ptr<This>;

    /// Default constructor for serialization
    BearingToKnownLandmarkFactor() = default;

    /**
     * @brief Constructor with Rot2 bearing
     * @param poseKey   Key for the robot pose variable
     * @param landmark  Known landmark position in world frame
     * @param measured  Measured bearing angle (as Rot2)
     * @param model     Noise model (must be 1-dimensional for bearing)
     */
    BearingToKnownLandmarkFactor(gtsam::Key poseKey,
                                  const gtsam::Point2& landmark,
                                  const gtsam::Rot2& measured,
                                  const gtsam::SharedNoiseModel& model);

    /**
     * @brief Constructor with bearing as double (radians)
     * @param poseKey       Key for the robot pose variable
     * @param landmark      Known landmark position in world frame
     * @param bearing_rad   Measured bearing angle in radians (0 = forward)
     * @param model         Noise model (must be 1-dimensional for bearing)
     */
    BearingToKnownLandmarkFactor(gtsam::Key poseKey,
                                  const gtsam::Point2& landmark,
                                  double bearing_rad,
                                  const gtsam::SharedNoiseModel& model);

    ~BearingToKnownLandmarkFactor() override = default;

    /// Clone for factor graph operations
    gtsam::NonlinearFactor::shared_ptr clone() const override;

    /// Print for debugging
    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

    /// Check equality
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;

    /**
     * @brief Evaluate error: measured - predicted bearing
     * @param pose  Current pose estimate
     * @param H     Optional Jacobian (1x3) with respect to pose
     * @return      1-dimensional error vector
     */
    gtsam::Vector evaluateError(
        const gtsam::Pose2& pose,
        gtsam::OptionalMatrixType H = nullptr) const override;

    /// Get the known landmark position
    const gtsam::Point2& landmark() const { return landmark_; }

    /// Get the measured bearing
    const gtsam::Rot2& measured() const { return measured_; }

private:
    gtsam::Point2 landmark_; ///< Known landmark position in world frame
    gtsam::Rot2 measured_;   ///< Measured bearing angle in robot frame
};

} // namespace decode
