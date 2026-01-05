/**
 * @file test_pose_estimator.cpp
 * @brief Unit tests for PoseEstimator
 */

#include <decode_estimator/pose_estimator.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using namespace decode;

// Wrap angle to [-pi, pi]
double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

class PoseEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default landmarks for testing
        landmarks_ = {
            {1, 0.0, 10.0},
            {2, 10.0, 10.0},
            {3, 10.0, 0.0},
            {4, 0.0, 0.0},
        };
    }

    std::vector<Landmark> landmarks_;
};

// Test initialization
TEST_F(PoseEstimatorTest, Initialization) {
    EstimatorConfig config;
    PoseEstimator estimator(config);

    EXPECT_FALSE(estimator.isInitialized());

    estimator.initialize(landmarks_, 1.0, 2.0, 0.5);

    EXPECT_TRUE(estimator.isInitialized());
    EXPECT_EQ(estimator.getCurrentPoseIndex(), 0);

    PoseEstimate est = estimator.getCurrentEstimate();
    EXPECT_NEAR(est.x, 1.0, 1e-6);
    EXPECT_NEAR(est.y, 2.0, 1e-6);
    EXPECT_NEAR(est.theta, 0.5, 1e-6);
}

// Test reset
TEST_F(PoseEstimatorTest, Reset) {
    EstimatorConfig config;
    PoseEstimator estimator(config);

    estimator.initialize(landmarks_, 1.0, 2.0, 0.5);
    EXPECT_TRUE(estimator.isInitialized());

    estimator.reset();
    EXPECT_FALSE(estimator.isInitialized());
}

// Test odometry processing
TEST_F(PoseEstimatorTest, OdometryProcessing) {
    EstimatorConfig config;
    PoseEstimator estimator(config);

    estimator.initialize(landmarks_, 0.0, 0.0, 0.0);

    // Move forward 1 meter
    OdometryMeasurement odom;
    odom.dx = 1.0;
    odom.dy = 0.0;
    odom.dtheta = 0.0;
    odom.timestamp = 0.1;

    estimator.processOdometry(odom);
    PoseEstimate est = estimator.update();

    EXPECT_NEAR(est.x, 1.0, 0.01);
    EXPECT_NEAR(est.y, 0.0, 0.01);
    EXPECT_NEAR(est.theta, 0.0, 0.01);
    EXPECT_EQ(estimator.getCurrentPoseIndex(), 1);
}

// Test odometry with rotation
TEST_F(PoseEstimatorTest, OdometryWithRotation) {
    EstimatorConfig config;
    PoseEstimator estimator(config);

    estimator.initialize(landmarks_, 0.0, 0.0, 0.0);

    // Rotate 90 degrees
    OdometryMeasurement odom1;
    odom1.dx = 0.0;
    odom1.dy = 0.0;
    odom1.dtheta = M_PI / 2;
    odom1.timestamp = 0.1;

    estimator.processOdometry(odom1);
    estimator.update();

    // Now move forward (should move in +Y direction)
    OdometryMeasurement odom2;
    odom2.dx = 1.0;
    odom2.dy = 0.0;
    odom2.dtheta = 0.0;
    odom2.timestamp = 0.2;

    estimator.processOdometry(odom2);
    PoseEstimate est = estimator.update();

    EXPECT_NEAR(est.x, 0.0, 0.05);
    EXPECT_NEAR(est.y, 1.0, 0.05);
    EXPECT_NEAR(est.theta, M_PI / 2, 0.05);
}

// Test bearing measurements
TEST_F(PoseEstimatorTest, BearingMeasurement) {
    EstimatorConfig config;
    config.default_bearing_sigma = 0.01;  // Tight noise
    config.default_distance_sigma = 0.1;
    PoseEstimator estimator(config);

    estimator.initialize(landmarks_, 5.0, 5.0, 0.0);

    // Add a small odometry step
    OdometryMeasurement odom;
    odom.dx = 0.01;
    odom.dy = 0.0;
    odom.dtheta = 0.0;
    odom.timestamp = 0.1;
    estimator.processOdometry(odom);

    // Add bearing to tag 4 (at origin)
    // True bearing from (5,5) to (0,0) is atan2(-5, -5) = -135 degrees
    // In robot frame (facing +X): -135 degrees - 0 = -135 degrees
    BearingMeasurement bearing;
    bearing.tag_id = 4;
    bearing.bearing_rad = -3 * M_PI / 4;  // -135 degrees
    bearing.uncertainty_rad = 0.01;
    bearing.timestamp = 0.1;

    estimator.addBearingMeasurement(bearing);

    DistanceMeasurement distance;
    distance.tag_id = 4;
    distance.distance_m = std::sqrt(5.0 * 5.0 + 5.0 * 5.0);
    distance.uncertainty_m = 0.1;
    distance.timestamp = 0.1;

    estimator.addDistanceMeasurement(distance);
    PoseEstimate est = estimator.update();

    // Position should still be close to (5, 5)
    EXPECT_NEAR(est.x, 5.0, 0.2);
    EXPECT_NEAR(est.y, 5.0, 0.2);
}

// Test that bearing measurements correct drift
TEST_F(PoseEstimatorTest, DriftCorrection) {
    EstimatorConfig config;
    config.odom_sigma_xy = 0.05;      // Relatively noisy odometry
    config.default_bearing_sigma = 0.02;  // Tight bearing
    config.default_distance_sigma = 0.1;
    PoseEstimator estimator(config);

    // Start at known position
    double start_x = 5.0;
    double start_y = 5.0;
    estimator.initialize(landmarks_, start_x, start_y, 0.0);

    // Simulate odometry drift: move and then return, but with accumulated error
    // True path: stay at (5, 5)
    // Odometry path: slowly drift away

    double true_x = start_x;
    double true_y = start_y;
    double true_theta = 0.0;

    for (int i = 0; i < 50; i++) {
        // Small forward motion with intentional drift
        OdometryMeasurement odom;
        odom.dx = 0.1;    // Move forward
        odom.dy = 0.005;  // Small lateral drift
        odom.dtheta = 0.01;  // Small rotational drift
        odom.timestamp = i * 0.1;

        estimator.processOdometry(odom);

        // Update true position (without drift)
        true_x += 0.1 * std::cos(true_theta);
        true_y += 0.1 * std::sin(true_theta);

        // Add bearing measurements every 10 steps
        if (i % 10 == 0) {
            for (const auto& lm : landmarks_) {
                double dx = lm.x - true_x;
                double dy = lm.y - true_y;
                double global_bearing = std::atan2(dy, dx);
                double robot_bearing = wrapAngle(global_bearing - true_theta);

                BearingMeasurement bearing;
                bearing.tag_id = lm.id;
                bearing.bearing_rad = robot_bearing;
                bearing.uncertainty_rad = 0.02;
                bearing.timestamp = i * 0.1;

                estimator.addBearingMeasurement(bearing);

                DistanceMeasurement distance;
                distance.tag_id = lm.id;
                distance.distance_m = std::sqrt(dx * dx + dy * dy);
                distance.uncertainty_m = 0.1;
                distance.timestamp = i * 0.1;

                estimator.addDistanceMeasurement(distance);
            }
        }

        estimator.update();
    }

    PoseEstimate final_est = estimator.getCurrentEstimate();

    // With bearing corrections, estimate should be close to true position
    double error = std::sqrt((final_est.x - true_x) * (final_est.x - true_x) +
                             (final_est.y - true_y) * (final_est.y - true_y));

    // Should be within 0.5 meters of truth (with drift corrections)
    EXPECT_LT(error, 0.5);
}

// Test getTrajectory
TEST_F(PoseEstimatorTest, GetTrajectory) {
    EstimatorConfig config;
    PoseEstimator estimator(config);

    estimator.initialize(landmarks_, 0.0, 0.0, 0.0);

    // Add a few odometry measurements
    for (int i = 0; i < 5; i++) {
        OdometryMeasurement odom;
        odom.dx = 0.1;
        odom.dy = 0.0;
        odom.dtheta = 0.0;
        odom.timestamp = i * 0.1;
        estimator.processOdometry(odom);
        estimator.update();
    }

    std::vector<PoseEstimate> trajectory = estimator.getTrajectory();

    // Should have 6 poses (initial + 5 odometry steps)
    EXPECT_EQ(trajectory.size(), 6);

    // First pose should be near origin
    EXPECT_NEAR(trajectory[0].x, 0.0, 0.01);
    EXPECT_NEAR(trajectory[0].y, 0.0, 0.01);

    // Last pose should be near (0.5, 0)
    EXPECT_NEAR(trajectory[5].x, 0.5, 0.05);
    EXPECT_NEAR(trajectory[5].y, 0.0, 0.05);
}

// Test getCurrentEstimateWithCovariance
TEST_F(PoseEstimatorTest, Covariance) {
    EstimatorConfig config;
    PoseEstimator estimator(config);

    estimator.initialize(landmarks_, 0.0, 0.0, 0.0);

    // Add some odometry
    OdometryMeasurement odom;
    odom.dx = 1.0;
    odom.dy = 0.0;
    odom.dtheta = 0.0;
    odom.timestamp = 0.1;
    estimator.processOdometry(odom);
    estimator.update();

    PoseEstimate est = estimator.getCurrentEstimateWithCovariance();

    EXPECT_TRUE(est.has_covariance);

    // Covariance should be positive semi-definite
    // Check diagonal elements are positive
    EXPECT_GT(est.covariance[0], 0);  // xx
    EXPECT_GT(est.covariance[4], 0);  // yy
    EXPECT_GT(est.covariance[8], 0);  // theta-theta
}

// Test landmark map
TEST_F(PoseEstimatorTest, LandmarkMap) {
    EstimatorConfig config;
    PoseEstimator estimator(config);

    estimator.initialize(landmarks_, 0.0, 0.0, 0.0);

    const LandmarkMap& map = estimator.getLandmarkMap();

    EXPECT_EQ(map.size(), 4);
    EXPECT_TRUE(map.hasLandmark(1));
    EXPECT_TRUE(map.hasLandmark(2));
    EXPECT_TRUE(map.hasLandmark(3));
    EXPECT_TRUE(map.hasLandmark(4));
    EXPECT_FALSE(map.hasLandmark(99));

    auto lm1 = map.getLandmark(1);
    EXPECT_TRUE(lm1.has_value());
    EXPECT_NEAR(lm1->x(), 0.0, 1e-6);
    EXPECT_NEAR(lm1->y(), 10.0, 1e-6);
}

// Test exception on uninitialized
TEST_F(PoseEstimatorTest, UninitializedException) {
    EstimatorConfig config;
    PoseEstimator estimator(config);

    OdometryMeasurement odom;
    odom.dx = 1.0;
    odom.dy = 0.0;
    odom.dtheta = 0.0;
    odom.timestamp = 0.1;

    EXPECT_THROW(estimator.processOdometry(odom), std::runtime_error);
    EXPECT_THROW(estimator.update(), std::runtime_error);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
