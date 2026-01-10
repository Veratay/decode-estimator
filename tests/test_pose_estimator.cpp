/**
 * @file test_pose_estimator.cpp
 * @brief Unit tests for PoseEstimator
 */

#include <decode_estimator/pose_estimator.hpp>
#include <decode_estimator/camera_model.hpp>

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

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
        // Default landmarks for testing (3D Poses)
        // Assume tags are vertical walls facing inwards
        landmarks_ = {
            // Tag 1: (0, 10, 0.5), facing South (-Y)
            {1, 0.0, 10.0, 0.5, -M_PI/2, 0.0, -M_PI/2, 0.2},
            // Tag 2: (10, 10, 0.5), facing West (-X)
            {2, 10.0, 10.0, 0.5, -M_PI/2, 0.0, M_PI, 0.2},
            // Tag 3: (10, 0, 0.5), facing North (+Y)
            {3, 10.0, 0.0, 0.5, -M_PI/2, 0.0, M_PI/2, 0.2},
            // Tag 4: (0, 0, 0.5), facing East (+X)
            {4, 0.0, 0.0, 0.5, -M_PI/2, 0.0, 0.0, 0.2},
        };
    }

    std::vector<Landmark> landmarks_;

    // Helper to generate tag measurement
    TagMeasurement generateMeasurement(const EstimatorConfig& config, 
                                       const Landmark& tag,
                                       double robot_x, double robot_y, double robot_theta,
                                       double time) {
        TagMeasurement meas;
        meas.tag_id = tag.id;
        meas.timestamp = time;
        meas.pixel_sigma = 1.0;
        meas.turret_yaw_rad = 0.0;

        // Robot Pose3
        gtsam::Pose3 robot_pose(gtsam::Rot3::Ypr(robot_theta, 0, 0),
                                gtsam::Point3(robot_x, robot_y, 0));
        
        // Extrinsics
        gtsam::Pose3 extrinsics(gtsam::Rot3::Ypr(config.camera_yaw, config.camera_pitch, config.camera_roll),
                                gtsam::Point3(config.camera_offset_x, config.camera_offset_y, config.camera_offset_z));
        
        gtsam::Pose3 camera_pose = robot_pose.compose(extrinsics);
        decode::CameraModel intrinsics;
        intrinsics.fx = config.fx;
        intrinsics.fy = config.fy;
        intrinsics.cx = config.cx;
        intrinsics.cy = config.cy;
        intrinsics.k1 = config.k1;
        intrinsics.k2 = config.k2;
        intrinsics.k3 = config.k3;
        intrinsics.p1 = config.p1;
        intrinsics.p2 = config.p2;

        // Tag Pose
        gtsam::Pose3 tag_pose(gtsam::Rot3::Ypr(tag.yaw, tag.pitch, tag.roll),
                              gtsam::Point3(tag.x, tag.y, tag.z));
        
        double s = tag.size / 2.0;
        std::vector<gtsam::Point3> corners_local = {
            {-s, -s, 0}, {s, -s, 0}, {s, s, 0}, {-s, s, 0}
        };

        for(auto& pt : corners_local) {
            gtsam::Point3 pt_world = tag_pose.transformFrom(pt);
            gtsam::Point2 uv;
            if (decode::projectPoint(camera_pose, intrinsics, pt_world, &uv)) {
                meas.corners.emplace_back(uv.x(), uv.y());
            }
        }
        return meas;
    }
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

    PoseEstimate est = estimator.processOdometry(odom);

    EXPECT_NEAR(est.x, 1.0, 0.01);
    EXPECT_NEAR(est.y, 0.0, 0.01);
    EXPECT_NEAR(est.theta, 0.0, 0.01);
    EXPECT_EQ(estimator.getCurrentPoseIndex(), 0);
}

// Test tag measurements
TEST_F(PoseEstimatorTest, TagMeasurementTest) {
    EstimatorConfig config;
    config.camera_offset_z = 0.5;
    config.camera_yaw = -M_PI/2.0;
    config.camera_roll = -M_PI/2.0;

    PoseEstimator estimator(config);

    estimator.initialize(landmarks_, 5.0, 5.0, 0.0);

    // Add a small odometry step
    OdometryMeasurement odom;
    odom.dx = 0.01;
    odom.dy = 0.0;
    odom.dtheta = 0.0;
    odom.timestamp = 0.1;
    estimator.processOdometry(odom);

    // Robot is at (5,5), facing +X. Tag 4 is at (0,0).
    // Let's turn robot around to face origin.
    
    estimator.initialize(landmarks_, 5.0, 0.0, M_PI); // At (5,0) facing (0,0) (West)
    
    // Tag 4 is at (0,0,0.5), facing East (+X).
    // Robot at (5,0,0), facing West (-X). They face each other. Distance 5m.
    
    auto meas = generateMeasurement(config, landmarks_[3], 5.0, 0.0, M_PI, 0.1);
    
    if (meas.corners.size() == 4) {
        estimator.addTagMeasurement(meas);
        PoseEstimate est = estimator.update();
        // Position should be close to (5, 0)
        EXPECT_NEAR(est.x, 5.0, 0.1);
        EXPECT_NEAR(est.y, 0.0, 0.1);
        EXPECT_NEAR(est.theta, M_PI, 0.05); // +/- PI
    }
}

// Test drift correction
TEST_F(PoseEstimatorTest, DriftCorrection) {
    EstimatorConfig config;
    config.odom_sigma_xy = 0.05;
    config.default_pixel_sigma = 1.0;
    config.camera_offset_z = 0.5;
    config.camera_yaw = -M_PI/2.0;
    config.camera_roll = -M_PI/2.0;

    PoseEstimator estimator(config);

    // Start at (5,0) facing West (-X) towards Tag 4 at origin
    double start_x = 5.0;
    double start_y = 0.0;
    double start_theta = M_PI;
    
    estimator.initialize(landmarks_, start_x, start_y, start_theta);

    double true_x = start_x;
    double true_y = start_y;
    double true_theta = start_theta;

    PoseEstimate last_est = estimator.getCurrentEstimate();

    for (int i = 0; i < 20; i++) {
        // Move towards tag
        double dx = 0.1; 
        
        OdometryMeasurement odom;
        odom.dx = dx;
        odom.dy = 0.005; // Drift
        odom.dtheta = 0.01; // Drift
        odom.timestamp = i * 0.1;

        estimator.processOdometry(odom);

        true_x += dx * std::cos(true_theta);
        true_y += dx * std::sin(true_theta);
        // True theta doesn't change much (ignoring drift for truth simulation simplicity here)

        if (i % 5 == 0) {
            auto meas = generateMeasurement(config, landmarks_[3], true_x, true_y, true_theta, i*0.1);
            if(meas.corners.size() == 4) {
                estimator.addTagMeasurement(meas);
                last_est = estimator.update();
            }
        }
    }
    
    // Expect error to be low
    // True y is 0. Odom drifted.
    EXPECT_NEAR(last_est.y, 0.0, 0.5);
}

// Test landmark map
TEST_F(PoseEstimatorTest, LandmarkMap) {
    EstimatorConfig config;
    PoseEstimator estimator(config);

    estimator.initialize(landmarks_, 0.0, 0.0, 0.0);

    const LandmarkMap& map = estimator.getLandmarkMap();

    EXPECT_EQ(map.size(), 4);
    EXPECT_TRUE(map.hasLandmark(1));

    auto lm1 = map.getLandmark(1);
    EXPECT_TRUE(lm1.has_value());
    EXPECT_NEAR(lm1->x, 0.0, 1e-6);
    EXPECT_NEAR(lm1->y, 10.0, 1e-6);
    EXPECT_NEAR(lm1->z, 0.5, 1e-6);
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
