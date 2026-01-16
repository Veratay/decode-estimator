/**
 * @file simple_localization.cpp
 * @brief Example demonstrating the TagSLAM-inspired Pose Estimator
 *
 * This example simulates a robot moving in a square pattern while detecting
 * AprilTags. It generates synthetic pixel corner measurements and feeds them
 * to the estimator to correct odometry drift.
 */

#include <decode_estimator/pose_estimator.hpp>
#include <decode_estimator/camera_model.hpp>

#include <gtsam/geometry/Pose3.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>

// Simulation parameters
constexpr double DT = 0.01;            // 100 Hz
constexpr double ROBOT_SPEED = 0.5;    // m/s
constexpr double TURN_RATE = 0.5;      // rad/s
constexpr int NUM_ITERATIONS = 1000;

// Noise parameters for simulation
constexpr double ODOM_NOISE_XY = 0.005;     // meters per step
constexpr double ODOM_NOISE_THETA = 0.002;  // radians per step
constexpr double PIXEL_NOISE = 1.0;         // pixels

// Detection parameters
constexpr int DETECTION_INTERVAL = 10;      // Check for tags every N steps

/// Wrap angle to [-pi, pi]
double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// Helper to create a tag facing a specific direction
decode::Landmark createTag(int32_t id, double x, double y, double z, double facing_yaw) {
    decode::Landmark lm;
    lm.id = id;
    lm.x = x;
    lm.y = y;
    lm.z = z;
    lm.size = 0.2; // 20cm tag
    
    // Standard tag frame: Z out, X right, Y down
    // We want Z to point in 'facing_yaw' direction
    // If facing_yaw = 0 (East), Z=(1,0,0).
    // If we use Ypr(yaw, -90, 0)?
    // Rot3::Ypr(y,p,r) -> Rz(y)*Ry(p)*Rx(r)
    // Local Z (0,0,1) -> Global vector
    // To point Z horizontally, pitch = -pi/2?
    // Let's assume tags are vertical.
    
    // Explicit rotation construction:
    // Z axis (normal): (cos(yaw), sin(yaw), 0)
    // Y axis (down): (0, 0, -1)
    // X axis (right): Y x Z = (-sin, cos, 0)
    
    gtsam::Rot3 R(
        -std::sin(facing_yaw), 0, std::cos(facing_yaw), // Col 1 (X)
         std::cos(facing_yaw), 0, std::sin(facing_yaw), // Col 2 (Y) ?? Wait X cross Y = Z
         0,                   -1, 0                     // Col 3 (Z) - No wait
    );
    // Let's stick to YPR and guess:
    // Yaw=facing, Pitch=-90deg (to bring Z down to horizon), Roll=-90 (to align X/Y)?
    
    // Simpler: Just set YPR such that Z points out.
    // If Yaw=0, Pitch=0, Roll=0: Z is Up.
    // Pitch -90 deg: Z becomes X. (Rot around Y).
    // Then X becomes -Z (Down).
    
    lm.yaw = facing_yaw;
    lm.pitch = -M_PI / 2.0; 
    lm.roll = -M_PI / 2.0; 
    
    return lm;
}

void simulateDetection(double true_x, double true_y, double true_theta,
                       const decode::Landmark& tag,
                       const decode::EstimatorConfig& config,
                       double time,
                       decode::PoseEstimator& estimator,
                       std::mt19937& gen,
                       std::normal_distribution<>& pixel_noise) {
    
    // Robot Pose3
    gtsam::Pose3 robot_pose(gtsam::Rot3::Ypr(true_theta, 0, 0),
                            gtsam::Point3(true_x, true_y, 0));
    
    // Extrinsics (Body -> Camera)
    gtsam::Pose3 extrinsics(gtsam::Rot3::Ypr(config.camera_yaw, config.camera_pitch, config.camera_roll),
                            gtsam::Point3(config.camera_offset_x, config.camera_offset_y, config.camera_offset_z));
    
    // Camera World Pose
    gtsam::Pose3 camera_pose = robot_pose.compose(extrinsics);
    
    // Camera Model
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
    
    // Tag Pose3
    gtsam::Pose3 tag_pose(gtsam::Rot3::Ypr(tag.yaw, tag.pitch, tag.roll),
                          gtsam::Point3(tag.x, tag.y, tag.z));
                          
    double s = tag.size / 2.0;
    std::vector<gtsam::Point3> corners_local = {
        {-s, -s, 0}, {s, -s, 0}, {s, s, 0}, {-s, s, 0}
    };
    
    decode::TagMeasurement meas;
    meas.tag_id = tag.id;
    meas.timestamp = time;
    meas.pixel_sigma = PIXEL_NOISE;
    meas.turret_yaw_rad = 0.0;
    
    bool visible = true;
    for (const auto& pt_local : corners_local) {
        gtsam::Point3 pt_world = tag_pose.transformFrom(pt_local);
        
        gtsam::Point2 px;
        if (!decode::projectPoint(camera_pose, intrinsics, pt_world, &px)) {
            visible = false;
            break;
        }

        // Add noise
        double u = px.x() + pixel_noise(gen);
        double v = px.y() + pixel_noise(gen);

        // Check bounds (simple check)
        if (u < 0 || u > 2*config.cx || v < 0 || v > 2*config.cy) {
            visible = false;
            break;
        }
        meas.corners.emplace_back(u, v);
    }
    
    if (visible && meas.corners.size() == 4) {
        estimator.addTagMeasurement(meas);
    }
}

int main() {
    std::cout << "=== TagSLAM-Inspired Pose Estimator Demo ===" << std::endl;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> odom_noise_xy(0, ODOM_NOISE_XY);
    std::normal_distribution<> odom_noise_theta(0, ODOM_NOISE_THETA);
    std::normal_distribution<> pixel_noise(0, PIXEL_NOISE);

    // Configure estimator
    decode::EstimatorConfig config;
    config.relinearize_threshold = 0.01;
    config.relinearize_skip = 1;
    config.odom_sigma_xy = 0.01;
    config.odom_sigma_theta = 0.005;
    config.default_pixel_sigma = 1.0;

    // Viewing angle-dependent noise (new feature)
    config.pixel_sigma_angle_k = 2.0;  // Noise increases with viewing angle

    // Spatial correlation downweighting (new feature)
    config.enable_spatial_correlation = true;
    config.correlation_distance_m = 0.3;  // Measurements within 0.3m are correlated
    config.correlation_downweight_factor = 2.0;  // Double noise for correlated measurements
    config.correlation_history_size = 100;

    // Bias correction (new feature)
    config.enable_bias_correction = true;
    config.radial_bias_k = 0.01;  // Radial bias coefficient
    
    // Camera Setup (VGA)
    config.fx = 800.0;
    config.fy = 800.0;
    config.cx = 320.0;
    config.cy = 240.0;
    config.camera_offset_z = 0.5; // Camera 0.5m off ground
    // Camera facing forward (X), Z up? No, Z forward, Y down.
    // Robot Frame: X fwd, Y left, Z up.
    // Camera Frame: Z fwd, X right, Y down.
    // Rotation Robot->Camera:
    // Z_c = X_r
    // X_c = -Y_r
    // Y_c = -Z_r
    // Yaw=-90, Pitch=0, Roll=-90?
    config.camera_yaw = -M_PI/2.0;
    config.camera_pitch = 0.0;
    config.camera_roll = -M_PI/2.0;

#if DECODE_ENABLE_RERUN
    config.enable_visualization = true;
#endif

    decode::PoseEstimator estimator(config);

    // Define Landmarks (Walls of a 10x10 room)
    std::vector<decode::Landmark> landmarks;
    // Tag 1: X=10, Y=5, facing -X (Yaw=PI)
    landmarks.push_back(createTag(1, 10.0, 5.0, 1.0, M_PI));
    // Tag 2: X=5, Y=10, facing -Y (Yaw=-PI/2)
    landmarks.push_back(createTag(2, 5.0, 10.0, 1.0, -M_PI/2.0));
    // Tag 3: X=0, Y=5, facing +X (Yaw=0)
    landmarks.push_back(createTag(3, 0.0, 5.0, 1.0, 0.0));
    // Tag 4: X=5, Y=0, facing +Y (Yaw=PI/2)
    landmarks.push_back(createTag(4, 5.0, 0.0, 1.0, M_PI/2.0));

    // Initialize
    double initial_x = 2.0;
    double initial_y = 2.0;
    double initial_theta = 0.0;
    estimator.initialize(landmarks, initial_x, initial_y, initial_theta);

    double true_x = initial_x;
    double true_y = initial_y;
    double true_theta = initial_theta;
    
    double odom_x = initial_x;
    double odom_y = initial_y;
    double odom_theta = initial_theta;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Step   | True Pose        | Odom-only        | Estimated        | Error" << std::endl;
    std::cout << "-------|------------------|------------------|------------------|------" << std::endl;

    double time = 0.0;

    for (int i = 0; i < NUM_ITERATIONS; i++) {
        time += DT;
        
        // Motion
        double cmd_v = ROBOT_SPEED;
        double cmd_omega = (i % 200 > 150) ? TURN_RATE : 0.0;
        
        // True Update
        true_x += cmd_v * DT * std::cos(true_theta);
        true_y += cmd_v * DT * std::sin(true_theta);
        true_theta = wrapAngle(true_theta + cmd_omega * DT);
        
        // Odom Update (Noisy)
        double dx_r = cmd_v * DT + odom_noise_xy(gen);
        double dy_r = odom_noise_xy(gen);
        double dth = cmd_omega * DT + odom_noise_theta(gen);
        
        odom_x += dx_r * std::cos(odom_theta) - dy_r * std::sin(odom_theta);
        odom_y += dx_r * std::sin(odom_theta) + dy_r * std::cos(odom_theta);
        odom_theta = wrapAngle(odom_theta + dth);
        
        decode::OdometryMeasurement odom{dx_r, dy_r, dth, time};
        estimator.processOdometry(odom);
        
        // Vision Update
        if (i % DETECTION_INTERVAL == 0) {
            for (const auto& lm : landmarks) {
                simulateDetection(true_x, true_y, true_theta, lm, config, time, estimator, gen, pixel_noise);
            }
        }
        
        decode::PoseEstimate est = estimator.update();
        
        if (i % 100 == 0) {
             double error = std::sqrt(std::pow(est.x - true_x, 2) + std::pow(est.y - true_y, 2));
             std::cout << std::setw(6) << i << " | "
                       << "(" << std::setw(5) << true_x << ", " << std::setw(5) << true_y << ") | "
                       << "(" << std::setw(5) << odom_x << ", " << std::setw(5) << odom_y << ") | "
                       << "(" << std::setw(5) << est.x << ", " << std::setw(5) << est.y << ") | "
                       << std::setw(5) << error << std::endl;
        }
    }
    
    return 0;
}
