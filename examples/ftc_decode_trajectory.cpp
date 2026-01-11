/**
 * @file ftc_decode_trajectory.cpp
 * @brief Example simulating an FTC robot on a Decode field with corner AprilTags
 *
 * The robot follows a waypoint trajectory while detecting AprilTags placed
 * at two adjacent corners of the field (one tag per corner).
 */

#include <decode_estimator/ekf_localizer.hpp>
#include <decode_estimator/pose_estimator.hpp>

#include <gtsam/geometry/Pose3.h>
#include <decode_estimator/camera_model.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

// Simulation parameters
constexpr double DT = 0.02;            // 50 Hz
constexpr double ROBOT_SPEED = 0.6;    // m/s
constexpr double TURN_GAIN = 2.0;      // Heading correction gain
constexpr int NUM_ITERATIONS = 20000;

// Noise parameters for simulation
constexpr double ODOM_NOISE_XY = 0.004;     // meters per step
constexpr double ODOM_NOISE_THETA = 0.002;  // radians per step
constexpr double PIXEL_NOISE = 2.0;         // pixels
constexpr double TURRET_YAW_AMPLITUDE = 0.8; // radians
constexpr double TURRET_YAW_RATE = 0.6;      // rad/s

// Detection parameters
constexpr int DETECTION_INTERVAL = 5;       // Check for tags every N steps

// Field parameters (FTC field is ~12ft => 3.66m)
constexpr double FIELD_SIZE = 3.66;         // meters
constexpr double CORNER_TAG_OFFSET = 0.35;  // meters

struct Waypoint {
    double x;
    double y;
};

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
    
    // Tag rotation: Z out (facing direction), Y down, X right
    lm.yaw = facing_yaw;
    lm.pitch = -M_PI / 2.0; // Rotate to bring Z from Up to Horizontal
    lm.roll = -M_PI / 2.0;  // Adjust roll
    
    return lm;
}

void simulateDetection(double true_x, double true_y, double true_theta,
                       const decode::Landmark& tag,
                       const decode::EstimatorConfig& config,
                       double turret_yaw,
                       double time,
                       decode::PoseEstimator& estimator,
                       decode::EkfLocalizer& ekf,
                       std::mt19937& gen,
                       std::normal_distribution<>& pixel_noise) {
    
    // Robot Pose3
    gtsam::Pose3 robot_pose(gtsam::Rot3::Ypr(true_theta, 0, 0),
                            gtsam::Point3(true_x, true_y, 0));
    
    // Turret Pose3
    gtsam::Pose3 turret_pose(gtsam::Rot3::Yaw(turret_yaw), gtsam::Point3(0,0,0));

    // Extrinsics (Body -> Camera)
    gtsam::Pose3 extrinsics(gtsam::Rot3::Ypr(config.camera_yaw, config.camera_pitch, config.camera_roll),
                            gtsam::Point3(config.camera_offset_x, config.camera_offset_y, config.camera_offset_z));
    
    // Camera World Pose
    gtsam::Pose3 camera_pose = robot_pose.compose(turret_pose).compose(extrinsics);
    
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
    meas.turret_yaw_rad = turret_yaw;
    
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
        ekf.addTagMeasurement(meas);
    }
}

int main() {
    std::cout << "=== FTC Decode Field Trajectory Demo ===" << std::endl;
    std::cout << "Simulating an FTC robot following a trajectory with AprilTag detection"
              << std::endl << std::endl;

    // Random number generator for noise
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
    
    config.fx = 800.0;
    config.fy = 800.0;
    config.cx = 320.0;
    config.cy = 240.0;
    config.camera_offset_z = 0.4;
    config.camera_yaw = -M_PI/2.0;
    config.camera_roll = -M_PI/2.0;

#if DECODE_ENABLE_RERUN
    config.enable_visualization = true;
    config.visualization_app_id = "ftc_decode_trajectory";
    std::cout << "Rerun visualization enabled" << std::endl;
#else
    std::cout << "Rerun visualization disabled" << std::endl;
#endif

    decode::PoseEstimator estimator(config);
    decode::EKFConfig ekf_config;
    ekf_config.prior_sigma_xy = config.prior_sigma_xy;
    ekf_config.prior_sigma_theta = config.prior_sigma_theta;
    ekf_config.odom_sigma_xy = config.odom_sigma_xy;
    ekf_config.odom_sigma_theta = config.odom_sigma_theta;
    ekf_config.default_pixel_sigma = config.default_pixel_sigma;
    ekf_config.fx = config.fx;
    ekf_config.fy = config.fy;
    ekf_config.cx = config.cx;
    ekf_config.cy = config.cy;
    ekf_config.k1 = config.k1;
    ekf_config.k2 = config.k2;
    ekf_config.k3 = config.k3;
    ekf_config.p1 = config.p1;
    ekf_config.p2 = config.p2;
    ekf_config.camera_offset_x = config.camera_offset_x;
    ekf_config.camera_offset_y = config.camera_offset_y;
    ekf_config.camera_offset_z = config.camera_offset_z;
    ekf_config.camera_roll = config.camera_roll;
    ekf_config.camera_pitch = config.camera_pitch;
    ekf_config.camera_yaw = config.camera_yaw;
    
    decode::EkfLocalizer ekf(ekf_config);

    // AprilTags: one per corner, but only on two adjacent corners
    std::vector<decode::Landmark> landmarks;
    // Tag 1: Top-Left Corner, facing Southeast? Or just South/East?
    // Let's put tags on the walls.
    // Tag 1 (id 1): Near (0,0)? No, corners are (0,0), (0,S), (S,S), (S,0).
    // Tag 1: Near (0, FIELD_SIZE).
    landmarks.push_back(createTag(1, CORNER_TAG_OFFSET, FIELD_SIZE, 0.5, -M_PI/2.0)); // Facing South (down Y)
    
    // Tag 2: Near (FIELD_SIZE, FIELD_SIZE).
    landmarks.push_back(createTag(2, FIELD_SIZE, FIELD_SIZE - CORNER_TAG_OFFSET, 0.5, M_PI)); // Facing West (neg X)

    std::cout << "Landmarks:" << std::endl;
    for (const auto& lm : landmarks) {
        std::cout << "  Tag " << lm.id << ": (" << lm.x << ", " << lm.y << ")"
                  << std::endl;
    }
    std::cout << std::endl;

    // Waypoint trajectory (clockwise loop around the field)
    std::vector<Waypoint> waypoints = {
        {0.6, 0.6},
        {0.6, FIELD_SIZE - 0.6},
        {FIELD_SIZE - 0.6, FIELD_SIZE - 0.6},
        {FIELD_SIZE - 0.6, 0.6},
        {FIELD_SIZE / 2.0, FIELD_SIZE / 2.0},
    };

    // Initialize with known starting pose
    double initial_x = waypoints.front().x;
    double initial_y = waypoints.front().y;
    double initial_theta = 0.0;  // Facing +X direction

    estimator.initialize(landmarks, initial_x, initial_y, initial_theta);
    ekf.initialize(landmarks, initial_x, initial_y, initial_theta);

    double true_x = initial_x;
    double true_y = initial_y;
    double true_theta = initial_theta;

    double odom_x = initial_x;
    double odom_y = initial_y;
    double odom_theta = initial_theta;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Starting simulation..." << std::endl;
    std::cout << "Step   | True Pose        | Odom-only        | Estimated        | EKF             | Error"
              << std::endl;
    std::cout << "-------|------------------|------------------|------------------|-----------------|------"
              << std::endl;

    double time = 0.0;
    size_t waypoint_index = 1;
    constexpr double waypoint_threshold = 0.15;
    constexpr double slow_radius = 0.5;

    for (int i = 0; i < NUM_ITERATIONS; i++) {
        time += DT;

        const Waypoint& target = waypoints[waypoint_index];
        double dx_target = target.x - true_x;
        double dy_target = target.y - true_y;
        double dist_target = std::sqrt(dx_target * dx_target + dy_target * dy_target);

        if (dist_target < waypoint_threshold) {
            waypoint_index = (waypoint_index + 1) % waypoints.size();
        }

        double target_heading = std::atan2(dy_target, dx_target);
        double heading_error = wrapAngle(target_heading - true_theta);

        double speed_scale = std::min(1.0, dist_target / slow_radius);
        double cmd_v = ROBOT_SPEED * speed_scale;
        double cmd_omega = TURN_GAIN * heading_error;

        double true_dx = cmd_v * DT * std::cos(true_theta);
        double true_dy = cmd_v * DT * std::sin(true_theta);
        double true_dtheta = cmd_omega * DT;

        true_x += true_dx;
        true_y += true_dy;
        true_theta = wrapAngle(true_theta + true_dtheta);

        double odom_dx_robot = cmd_v * DT + odom_noise_xy(gen);
        double odom_dy_robot = odom_noise_xy(gen);
        double odom_dtheta = cmd_omega * DT + odom_noise_theta(gen);

        odom_x += odom_dx_robot * std::cos(odom_theta) - odom_dy_robot * std::sin(odom_theta);
        odom_y += odom_dx_robot * std::sin(odom_theta) + odom_dy_robot * std::cos(odom_theta);
        odom_theta = wrapAngle(odom_theta + odom_dtheta);

        decode::OdometryMeasurement odom;
        odom.dx = odom_dx_robot;
        odom.dy = odom_dy_robot;
        odom.dtheta = odom_dtheta;
        odom.timestamp = time;
        estimator.processOdometry(odom);
        ekf.processOdometry(odom);

        double turret_yaw = TURRET_YAW_AMPLITUDE * std::sin(TURRET_YAW_RATE * time);

        if (i % DETECTION_INTERVAL == 0) {
            for (const auto& lm : landmarks) {
                simulateDetection(true_x, true_y, true_theta, lm, config, turret_yaw, time, estimator, ekf, gen, pixel_noise);
            }
        }

        decode::PoseEstimate est = estimator.update();
        decode::PoseEstimate ekf_est = ekf.getCurrentEstimate();
        decode::PoseEstimate post_est = estimator.getPostProcessedEstimate();
        decode::PoseEstimate post_ekf = ekf.getPostProcessedEstimate();

        decode::PoseEstimate true_pose;
        true_pose.x = true_x;
        true_pose.y = true_y;
        true_pose.theta = true_theta;
        true_pose.timestamp = time;
        true_pose.has_covariance = false;

        decode::PoseEstimate odom_pose;
        odom_pose.x = odom_x;
        odom_pose.y = odom_y;
        odom_pose.theta = odom_theta;
        odom_pose.timestamp = time;
        odom_pose.has_covariance = false;

        double position_error = std::sqrt((est.x - true_x) * (est.x - true_x) +
                                          (est.y - true_y) * (est.y - true_y));
        double odom_error = std::sqrt((odom_x - true_x) * (odom_x - true_x) +
                                      (odom_y - true_y) * (odom_y - true_y));
        double ekf_error = std::sqrt((ekf_est.x - true_x) * (ekf_est.x - true_x) +
                                     (ekf_est.y - true_y) * (ekf_est.y - true_y));
        double post_error = std::sqrt((post_est.x - true_x) * (post_est.x - true_x) +
                                      (post_est.y - true_y) * (post_est.y - true_y));
        double post_ekf_error = std::sqrt((post_ekf.x - true_x) * (post_ekf.x - true_x) +
                                          (post_ekf.y - true_y) * (post_ekf.y - true_y));

        estimator.logTimeSeriesMetrics(i,
                                       est,
                                       true_pose,
                                       odom_pose,
                                       ekf_est,
                                       post_est,
                                       post_ekf,
                                       position_error,
                                       odom_error,
                                       ekf_error,
                                       post_error,
                                       post_ekf_error);

        if (i % 100 == 0 || i == NUM_ITERATIONS - 1) {
            std::cout << std::setw(6) << i << " | "
                      << "(" << std::setw(5) << true_x << ", " << std::setw(5) << true_y << ") | "
                      << "(" << std::setw(5) << odom_x << ", " << std::setw(5) << odom_y << ") | "
                      << "(" << std::setw(5) << est.x << ", " << std::setw(5) << est.y << ") | "
                      << "(" << std::setw(5) << ekf_est.x << ", " << std::setw(5) << ekf_est.y << ") | "
                      << std::setw(5) << position_error << " (odom: " << odom_error
                      << ", ekf: " << ekf_error << ")"
                      << std::endl;
        }

#if DECODE_ENABLE_RERUN
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
    }

    std::cout << std::endl;

    decode::PoseEstimate final_est = estimator.getCurrentEstimate();
    double final_error = std::sqrt((final_est.x - true_x) * (final_est.x - true_x) +
                                   (final_est.y - true_y) * (final_est.y - true_y));
    double final_odom_error = std::sqrt((odom_x - true_x) * (odom_x - true_x) +
                                        (odom_y - true_y) * (odom_y - true_y));
    decode::PoseEstimate final_ekf = ekf.getCurrentEstimate();
    double final_ekf_error = std::sqrt((final_ekf.x - true_x) * (final_ekf.x - true_x) +
                                       (final_ekf.y - true_y) * (final_ekf.y - true_y));

    std::cout << "=== Final Results ===" << std::endl;
    std::cout << "True final pose:      (" << true_x << ", " << true_y << ", " << true_theta << ")"
              << std::endl;
    std::cout << "Estimated final pose: (" << final_est.x << ", " << final_est.y << ", "
              << final_est.theta << ")" << std::endl;
    std::cout << "EKF final pose:       (" << final_ekf.x << ", " << final_ekf.y << ", "
              << final_ekf.theta << ")" << std::endl;
    std::cout << "Odometry-only pose:   (" << odom_x << ", " << odom_y << ", " << odom_theta << ")"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Estimator position error: " << final_error << " m" << std::endl;
    std::cout << "EKF position error:       " << final_ekf_error << " m" << std::endl;
    std::cout << "Odometry-only error:      " << final_odom_error << " m" << std::endl;
    std::cout << "Average solve time:       " << estimator.getAverageSolveTimeMs() << " ms" << std::endl;
    std::cout << "Horizon size:             " << estimator.getHorizonSize() << " poses" << std::endl;
    std::cout << "Improvement:              " << (final_odom_error - final_error) << " m ("
              << ((final_odom_error - final_error) / final_odom_error * 100) << "% reduction)"
              << std::endl;

#if DECODE_ENABLE_RERUN
    std::cout << std::endl;
    std::cout << "Visualization is open in Rerun viewer. Press Ctrl+C to exit." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(30));
#endif

    return 0;
}
