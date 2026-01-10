/**
 * @file simple_localization.cpp
 * @brief Example demonstrating the GTSAM iSAM2 pose estimator
 *
 * This example simulates a robot moving in a square pattern while detecting
 * AprilTags at known positions. It demonstrates how odometry drift is
 * corrected by fusing bearing measurements to known landmarks.
 */

#include <decode_estimator/pose_estimator.hpp>

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
constexpr double BEARING_NOISE = 0.03;      // radians (~2 degrees)
constexpr double DISTANCE_NOISE = 0.15;     // meters (less confident than bearing)

// Detection parameters
constexpr double DETECTION_RANGE = 5.0;     // meters
constexpr int DETECTION_INTERVAL = 10;      // Check for tags every N steps

/// Wrap angle to [-pi, pi]
double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

/// Compute bearing from robot pose to landmark (in robot frame)
double computeBearing(double robot_x, double robot_y, double robot_theta,
                      double landmark_x, double landmark_y) {
    double dx = landmark_x - robot_x;
    double dy = landmark_y - robot_y;
    double global_bearing = std::atan2(dy, dx);
    return wrapAngle(global_bearing - robot_theta);
}

/// Compute distance from robot to landmark
double computeDistance(double robot_x, double robot_y,
                       double landmark_x, double landmark_y) {
    double dx = landmark_x - robot_x;
    double dy = landmark_y - robot_y;
    return std::sqrt(dx * dx + dy * dy);
}

int main() {
    std::cout << "=== GTSAM iSAM2 Pose Estimator Demo ===" << std::endl;
    std::cout << "Simulating robot moving in a pattern while detecting AprilTags" << std::endl;
    std::cout << std::endl;

    // Random number generator for noise
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> odom_noise_xy(0, ODOM_NOISE_XY);
    std::normal_distribution<> odom_noise_theta(0, ODOM_NOISE_THETA);
    std::normal_distribution<> bearing_noise(0, BEARING_NOISE);
    std::normal_distribution<> distance_noise(0, DISTANCE_NOISE);

    // Configure estimator
    decode::EstimatorConfig config;
    config.relinearize_threshold = 0.01;
    config.relinearize_skip = 1;
    config.odom_sigma_xy = 0.01;          // Slightly conservative
    config.odom_sigma_theta = 0.005;
    config.default_bearing_sigma = 0.05;  // ~3 degrees
    config.default_distance_sigma = 0.2;  // Larger uncertainty than bearing

#if DECODE_ENABLE_RERUN
    config.enable_visualization = true;
    config.visualization_app_id = "robot_localization_demo";
    std::cout << "Rerun visualization enabled" << std::endl;
#else
    std::cout << "Rerun visualization disabled" << std::endl;
#endif

    // Create estimator
    decode::PoseEstimator estimator(config);

    // Define AprilTag landmarks (corners of a 10x10 field)
    std::vector<decode::Landmark> landmarks = {
        {1, 0.0, 10.0},   // Tag 1 at top-left
        {2, 10.0, 10.0},  // Tag 2 at top-right
        {3, 10.0, 0.0},   // Tag 3 at bottom-right
        {4, 0.0, 0.0},    // Tag 4 at bottom-left
        {5, 5.0, 5.0},    // Tag 5 at center
    };

    std::cout << "Landmarks:" << std::endl;
    for (const auto& lm : landmarks) {
        std::cout << "  Tag " << lm.id << ": (" << lm.x << ", " << lm.y << ")" << std::endl;
    }
    std::cout << std::endl;

    // Initialize with known starting pose (near bottom-left)
    double initial_x = 1.0;
    double initial_y = 1.0;
    double initial_theta = 0.0;  // Facing +X direction

    estimator.initialize(landmarks, initial_x, initial_y, initial_theta);

    // Ground truth pose (what the robot actually is)
    double true_x = initial_x;
    double true_y = initial_y;
    double true_theta = initial_theta;

    // Odometry-only estimate (for comparison - shows drift)
    double odom_x = initial_x;
    double odom_y = initial_y;
    double odom_theta = initial_theta;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Starting simulation..." << std::endl;
    std::cout << "Step   | True Pose        | Odom-only        | Estimated        | Error" << std::endl;
    std::cout << "-------|------------------|------------------|------------------|------" << std::endl;

    double time = 0.0;

    for (int i = 0; i < NUM_ITERATIONS; i++) {
        time += DT;

        // Determine commanded motion (move in a square-ish pattern)
        double cmd_v = ROBOT_SPEED;  // Forward velocity
        double cmd_omega = 0.0;       // Angular velocity

        // Turn at certain intervals to create a pattern
        if (i > 0 && i % 200 == 0) {
            cmd_omega = TURN_RATE;  // Start turning
        }
        if (i > 0 && i % 200 > 20 && i % 200 < 30) {
            cmd_omega = TURN_RATE;  // Continue turning
        }

        // Compute true motion
        double true_dx = cmd_v * DT * std::cos(true_theta);
        double true_dy = cmd_v * DT * std::sin(true_theta);
        double true_dtheta = cmd_omega * DT;

        // Update true pose
        true_x += true_dx;
        true_y += true_dy;
        true_theta = wrapAngle(true_theta + true_dtheta);

        // Generate noisy odometry (in robot frame)
        double odom_dx_robot = cmd_v * DT + odom_noise_xy(gen);
        double odom_dy_robot = odom_noise_xy(gen);  // Lateral noise
        double odom_dtheta = cmd_omega * DT + odom_noise_theta(gen);

        // Update odometry-only estimate (for comparison)
        odom_x += odom_dx_robot * std::cos(odom_theta) - odom_dy_robot * std::sin(odom_theta);
        odom_y += odom_dx_robot * std::sin(odom_theta) + odom_dy_robot * std::cos(odom_theta);
        odom_theta = wrapAngle(odom_theta + odom_dtheta);

        // Create odometry measurement
        decode::OdometryMeasurement odom;
        odom.dx = odom_dx_robot;
        odom.dy = odom_dy_robot;
        odom.dtheta = odom_dtheta;
        odom.timestamp = time;

        // Process odometry
        estimator.processOdometry(odom);

        // Check for AprilTag detections periodically
        if (i % DETECTION_INTERVAL == 0) {
            for (const auto& lm : landmarks) {
                double dist = computeDistance(true_x, true_y, lm.x, lm.y);

                // Only detect tags within range
                if (dist < DETECTION_RANGE) {
                    // Compute true bearing and add noise
                    double true_bearing = computeBearing(true_x, true_y, true_theta, lm.x, lm.y);
                    double measured_bearing = true_bearing + bearing_noise(gen);
                    double measured_distance = dist + distance_noise(gen);

                    decode::BearingMeasurement bearing;
                    bearing.tag_id = lm.id;
                    bearing.bearing_rad = measured_bearing;
                    bearing.turret_yaw_rad = 0.0;
                    bearing.uncertainty_rad = BEARING_NOISE * 1.5;  // Slightly conservative
                    bearing.timestamp = time;

                    estimator.addBearingMeasurement(bearing);

                    decode::DistanceMeasurement distance;
                    distance.tag_id = lm.id;
                    distance.distance_m = measured_distance;
                    distance.turret_yaw_rad = 0.0;
                    distance.uncertainty_m = DISTANCE_NOISE * 1.5;
                    distance.timestamp = time;

                    estimator.addDistanceMeasurement(distance);
                }
            }
        }

        // Perform update
        decode::PoseEstimate est = estimator.update();

        // Print progress every 100 iterations
        if (i % 100 == 0 || i == NUM_ITERATIONS - 1) {
            double error = std::sqrt((est.x - true_x) * (est.x - true_x) +
                                     (est.y - true_y) * (est.y - true_y));
            double odom_error = std::sqrt((odom_x - true_x) * (odom_x - true_x) +
                                          (odom_y - true_y) * (odom_y - true_y));

            std::cout << std::setw(6) << i << " | "
                      << "(" << std::setw(5) << true_x << ", " << std::setw(5) << true_y << ") | "
                      << "(" << std::setw(5) << odom_x << ", " << std::setw(5) << odom_y << ") | "
                      << "(" << std::setw(5) << est.x << ", " << std::setw(5) << est.y << ") | "
                      << std::setw(5) << error << " (odom: " << odom_error << ")"
                      << std::endl;
        }

#if DECODE_ENABLE_RERUN
        // Small delay for visualization
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
    }

    std::cout << std::endl;

    // Final statistics
    decode::PoseEstimate final_est = estimator.getCurrentEstimate();
    double final_error = std::sqrt((final_est.x - true_x) * (final_est.x - true_x) +
                                   (final_est.y - true_y) * (final_est.y - true_y));
    double final_odom_error = std::sqrt((odom_x - true_x) * (odom_x - true_x) +
                                        (odom_y - true_y) * (odom_y - true_y));

    std::cout << "=== Final Results ===" << std::endl;
    std::cout << "True final pose:      (" << true_x << ", " << true_y << ", " << true_theta << ")" << std::endl;
    std::cout << "Estimated final pose: (" << final_est.x << ", " << final_est.y << ", " << final_est.theta << ")" << std::endl;
    std::cout << "Odometry-only pose:   (" << odom_x << ", " << odom_y << ", " << odom_theta << ")" << std::endl;
    std::cout << std::endl;
    std::cout << "Estimator position error: " << final_error << " m" << std::endl;
    std::cout << "Odometry-only error:      " << final_odom_error << " m" << std::endl;
    std::cout << "Average solve time:       " << estimator.getAverageSolveTimeMs() << " ms" << std::endl;
    std::cout << "Horizon size:             " << estimator.getHorizonSize() << " poses" << std::endl;
    std::cout << "Improvement:              " << (final_odom_error - final_error) << " m ("
              << ((final_odom_error - final_error) / final_odom_error * 100) << "% reduction)" << std::endl;

    // Get estimate with covariance
    decode::PoseEstimate est_with_cov = estimator.getCurrentEstimateWithCovariance();
    if (est_with_cov.has_covariance) {
        std::cout << std::endl;
        std::cout << "Position uncertainty (1-sigma):" << std::endl;
        std::cout << "  x:     " << std::sqrt(est_with_cov.covariance[0]) << " m" << std::endl;
        std::cout << "  y:     " << std::sqrt(est_with_cov.covariance[4]) << " m" << std::endl;
        std::cout << "  theta: " << std::sqrt(est_with_cov.covariance[8]) << " rad" << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Total poses in graph: " << estimator.getCurrentPoseIndex() + 1 << std::endl;

#if DECODE_ENABLE_RERUN
    std::cout << std::endl;
    std::cout << "Visualization is open in Rerun viewer. Press Ctrl+C to exit." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(30));
#endif

    return 0;
}
