/**
 * @file ftc_decode_trajectory.cpp
 * @brief Example simulating an FTC robot on a Decode field with corner AprilTags
 *
 * The robot follows a waypoint trajectory while detecting AprilTags placed
 * at two adjacent corners of the field (one tag per corner).
 */

#include <decode_estimator/ekf_localizer.hpp>
#include <decode_estimator/pose_estimator.hpp>

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
constexpr double BEARING_NOISE = 0.025;     // radians (~1.4 degrees)
constexpr double DISTANCE_NOISE = 0.12;     // meters (less confident than bearing)
constexpr double TURRET_YAW_AMPLITUDE = 0.8; // radians
constexpr double TURRET_YAW_RATE = 0.6;      // rad/s

// Detection parameters
constexpr double DETECTION_RANGE = 3.0;     // meters
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
    std::cout << "=== FTC Decode Field Trajectory Demo ===" << std::endl;
    std::cout << "Simulating an FTC robot following a trajectory with AprilTag detection"
              << std::endl << std::endl;

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
    config.odom_sigma_xy = 0.01;
    config.odom_sigma_theta = 0.005;
    config.default_bearing_sigma = 0.05;
    config.default_distance_sigma = 0.2;

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
    ekf_config.default_bearing_sigma = config.default_bearing_sigma;
    ekf_config.default_distance_sigma = config.default_distance_sigma;
    decode::EkfLocalizer ekf(ekf_config);

    // AprilTags: one per corner, but only on two adjacent corners
    std::vector<decode::Landmark> landmarks = {
        {1, CORNER_TAG_OFFSET, CORNER_TAG_OFFSET},
        {2, FIELD_SIZE - CORNER_TAG_OFFSET, CORNER_TAG_OFFSET},
    };

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

    std::cout << "Trajectory waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i + 1 << ": (" << waypoints[i].x << ", "
                  << waypoints[i].y << ")" << std::endl;
    }
    std::cout << std::endl;

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
                double dist = computeDistance(true_x, true_y, lm.x, lm.y);
                if (dist < DETECTION_RANGE) {
                    double true_bearing = computeBearing(true_x, true_y, true_theta, lm.x, lm.y);
                    double turret_bearing = wrapAngle(true_bearing - turret_yaw);
                    double measured_bearing = turret_bearing + bearing_noise(gen);
                    double measured_distance = dist + distance_noise(gen);

                    decode::BearingMeasurement bearing;
                    bearing.tag_id = lm.id;
                    bearing.bearing_rad = measured_bearing;
                    bearing.turret_yaw_rad = turret_yaw;
                    bearing.uncertainty_rad = BEARING_NOISE * 1.5;
                    bearing.timestamp = time;

                    estimator.addBearingMeasurement(bearing);
                    ekf.addBearingMeasurement(bearing);

                    decode::DistanceMeasurement distance;
                    distance.tag_id = lm.id;
                    distance.distance_m = measured_distance;
                    distance.turret_yaw_rad = turret_yaw;
                    distance.uncertainty_m = DISTANCE_NOISE * 1.5;
                    distance.timestamp = time;

                    estimator.addDistanceMeasurement(distance);
                    ekf.addDistanceMeasurement(distance);
                }
            }
        }

        decode::PoseEstimate est = estimator.update();
        decode::PoseEstimate ekf_est = ekf.getCurrentEstimate();

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

        estimator.logTimeSeriesMetrics(i,
                                       est,
                                       true_pose,
                                       odom_pose,
                                       ekf_est,
                                       position_error,
                                       odom_error,
                                       ekf_error);

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
