/**
 * @file test_range_factor.cpp
 * @brief Unit tests for RangeToKnownLandmarkFactor
 */

#include <decode_estimator/range_factor.hpp>

#include <gtsam/linear/NoiseModel.h>
#include <gtest/gtest.h>

using namespace decode;

TEST(RangeFactorTest, ZeroErrorAtTrueRange) {
    gtsam::Point2 landmark(3.0, 4.0);
    gtsam::Pose2 pose(0.0, 0.0, 0.0);
    double measured_range = 5.0;

    auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
    RangeToKnownLandmarkFactor factor(0, landmark, measured_range, noise);

    gtsam::Vector error = factor.evaluateError(pose);
    EXPECT_NEAR(error(0), 0.0, 1e-6);
}

TEST(RangeFactorTest, RangeErrorSign) {
    gtsam::Point2 landmark(3.0, 4.0);
    gtsam::Pose2 pose(0.0, 0.0, 0.0);
    double measured_range = 4.0;

    auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
    RangeToKnownLandmarkFactor factor(0, landmark, measured_range, noise);

    gtsam::Vector error = factor.evaluateError(pose);
    EXPECT_NEAR(error(0), -1.0, 1e-6);
}
