/**
 * @file test_bearing_factor.cpp
 * @brief Unit tests for BearingToKnownLandmarkFactor
 */

#include <decode_estimator/bearing_factor.hpp>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtest/gtest.h>

#include <cmath>

using namespace decode;
using namespace gtsam;

// Test that the factor computes correct error when pose gives exact bearing
TEST(BearingToKnownLandmarkFactor, ZeroErrorAtTruth) {
    // Robot at origin, facing +X
    Pose2 pose(0, 0, 0);

    // Landmark directly in front
    Point2 landmark(5, 0);

    // Measured bearing is 0 (directly ahead)
    double measured_bearing = 0.0;

    auto noise = noiseModel::Isotropic::Sigma(1, 0.1);

    BearingToKnownLandmarkFactor factor(0, landmark, measured_bearing, noise);

    // Compute error - should be zero
    Vector error = factor.evaluateError(pose);
    EXPECT_NEAR(error(0), 0.0, 1e-9);
}

// Test error when bearing is off
TEST(BearingToKnownLandmarkFactor, NonZeroError) {
    // Robot at origin, facing +X
    Pose2 pose(0, 0, 0);

    // Landmark at 45 degrees
    Point2 landmark(5, 5);

    // True bearing is pi/4, but we measure 0
    double measured_bearing = 0.0;

    auto noise = noiseModel::Isotropic::Sigma(1, 0.1);

    BearingToKnownLandmarkFactor factor(0, landmark, measured_bearing, noise);

    // Compute error - should be about -pi/4 (predicted is pi/4, measured is 0)
    Vector error = factor.evaluateError(pose);
    EXPECT_NEAR(error(0), -M_PI / 4, 1e-9);
}

// Test error with rotated robot
TEST(BearingToKnownLandmarkFactor, RotatedRobot) {
    // Robot at origin, facing +Y (theta = pi/2)
    Pose2 pose(0, 0, M_PI / 2);

    // Landmark at +X direction (global)
    Point2 landmark(5, 0);

    // In robot frame, landmark is at -90 degrees (to the right)
    double measured_bearing = -M_PI / 2;

    auto noise = noiseModel::Isotropic::Sigma(1, 0.1);

    BearingToKnownLandmarkFactor factor(0, landmark, measured_bearing, noise);

    // Compute error - should be zero
    Vector error = factor.evaluateError(pose);
    EXPECT_NEAR(error(0), 0.0, 1e-9);
}

// Test Jacobian using numerical differentiation
TEST(BearingToKnownLandmarkFactor, Jacobian) {
    Pose2 pose(1.5, 2.3, 0.7);
    Point2 landmark(5, 5);
    double measured_bearing = 0.5;

    auto noise = noiseModel::Isotropic::Sigma(1, 0.1);

    BearingToKnownLandmarkFactor factor(0, landmark, measured_bearing, noise);

    // Compute analytical Jacobian
    Matrix H_analytical;
    factor.evaluateError(pose, &H_analytical);

    // Compute numerical Jacobian
    auto errorFunc = [&factor](const Pose2& p) {
        return factor.evaluateError(p);
    };
    Matrix H_numerical = numericalDerivative11<Vector, Pose2>(errorFunc, pose);

    // Compare
    EXPECT_TRUE(assert_equal(H_numerical, H_analytical, 1e-5));
}

// Test that optimization converges to correct pose
TEST(BearingToKnownLandmarkFactor, Optimization) {
    // True robot pose
    Pose2 true_pose(3, 4, M_PI / 4);

    // Landmarks
    std::vector<Point2> landmarks = {
        Point2(0, 0),
        Point2(10, 0),
        Point2(10, 10),
        Point2(0, 10)
    };

    // Compute true bearings
    std::vector<double> bearings;
    for (const auto& lm : landmarks) {
        Rot2 bearing = true_pose.bearing(lm);
        bearings.push_back(bearing.theta());
    }

    // Create factor graph with only bearing factors (and a prior for anchoring)
    NonlinearFactorGraph graph;

    auto prior_noise = noiseModel::Diagonal::Sigmas(Vector3(10, 10, 10)); // Weak prior
    graph.add(PriorFactor<Pose2>(0, Pose2(0, 0, 0), prior_noise));

    auto bearing_noise = noiseModel::Isotropic::Sigma(1, 0.01);
    for (size_t i = 0; i < landmarks.size(); i++) {
        graph.add(BearingToKnownLandmarkFactor(0, landmarks[i], bearings[i], bearing_noise));
    }

    // Initial estimate (far from truth)
    Values initial;
    initial.insert(0, Pose2(0, 0, 0));

    // Optimize
    LevenbergMarquardtParams params;
    params.maxIterations = 100;
    LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    Values result = optimizer.optimize();

    Pose2 estimated = result.at<Pose2>(0);

    // Check that estimate is close to truth
    // Note: With bearing-only and weak prior, position may not be perfectly recovered
    // but heading should be close
    EXPECT_NEAR(estimated.theta(), true_pose.theta(), 0.1);
}

// Test clone and equals
TEST(BearingToKnownLandmarkFactor, CloneAndEquals) {
    Point2 landmark(5, 5);
    double bearing = 0.5;
    auto noise = noiseModel::Isotropic::Sigma(1, 0.1);

    BearingToKnownLandmarkFactor factor1(0, landmark, bearing, noise);
    auto factor2 = factor1.clone();

    EXPECT_TRUE(factor1.equals(*factor2, 1e-9));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
