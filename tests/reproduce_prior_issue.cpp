#include "decode_estimator/pose_estimator.hpp"
#include <gtsam/geometry/Pose3.h>
#include <iostream>
#include <vector>
#include <cmath>

using namespace decode;

int main() {
    // 1. Setup Config
    EstimatorConfig config;
    config.prior_sigma_xy = 100.0; // Huge uncertainty
    config.prior_sigma_theta = 10.0;
    config.default_pixel_sigma = 1.0; // Tight vision
    config.fx = 1000;
    config.fy = 1000;
    config.cx = 500;
    config.cy = 500;
    
    // Disable some features for isolation
    config.enable_post_process = false;
    config.enable_tag_gating = false;
    config.enable_cheirality_check = false;

    PoseEstimator estimator(config);

    // 2. Setup Landmarks
    // Tag 1 at (5, 0, 1), facing towards origin (yaw=pi)
    std::vector<Landmark> landmarks;
    Landmark lm;
    lm.id = 1;
    lm.x = 5.0; lm.y = 0.0; lm.z = 1.0;
    lm.roll = 0; lm.pitch = 0; lm.yaw = M_PI; // Facing -X
    lm.size = 0.2;
    landmarks.push_back(lm);

    // 3. Initialize at WRONG position
    // True robot pos: (0,0,0)
    // Initial guess: (-5, 0, 0)
    double initial_x = -5.0;
    double initial_y = 0.0;
    double initial_theta = 0.0;
    
    std::cout << "Initializing at (" << initial_x << ", " << initial_y << ")\n";
    estimator.initialize(landmarks, initial_x, initial_y, initial_theta);

    // 4. Simulate One Step
    // Robot is actually at (0,0,0). Camera is at robot center (offset 0).
    // Tag 1 is at (5,0,1).
    // In camera frame (Robot=Camera), Tag is at (5,0,1).
    // Project tag corners.
    
    // We can use the helper in estimator to get corners if we were at truth, 
    // but here we just manually compute ideal pixel coords for a robot at (0,0,0).
    // Camera is at (0,0,0). Tag is at (5,0,1).
    // Tag corners local: (+-0.1, +-0.1, 0)
    // Tag corners world: 
    //   Center: (5,0,1). 
    //   Rot: YPR(pi,0,0). X_tag = -X_world. Y_tag = -Y_world.
    //   TL (-0.1, -0.1) -> World (5 - (-0.1), 0 - (-0.1), 1) = (5.1, 0.1, 1)
    //   Wait, Rotation is pi around Z.
    //   Rot Z(pi) = [-1 0 0; 0 -1 0; 0 0 1].
    //   Local (x,y,0) -> World (5-x, -y, 1).
    //   TL (-0.1, -0.1) -> (5.1, 0.1, 1)
    //   TR ( 0.1, -0.1) -> (4.9, 0.1, 1)
    //   BR ( 0.1,  0.1) -> (4.9, -0.1, 1)
    //   BL (-0.1,  0.1) -> (5.1, -0.1, 1)
    
    std::vector<gtsam::Point3> corners_world;
    corners_world.emplace_back(5.1, 0.1, 1.0);
    corners_world.emplace_back(4.9, 0.1, 1.0);
    corners_world.emplace_back(4.9, -0.1, 1.0);
    corners_world.emplace_back(5.1, -0.1, 1.0);
    
    std::vector<std::pair<double,double>> corners_px;
    for(const auto& p : corners_world) {
        // Project (x,y,z) -> (u,v)
        // x_cam = x, y_cam = y, z_cam = z (Identity extrinsics)
        // u = fx * x/z + cx
        // v = fy * y/z + cy
        double u = 1000.0 * p.x() / p.z() + 500.0;
        double v = 1000.0 * p.y() / p.z() + 500.0;
        corners_px.push_back({u, v});
    }

    // Add Measurement
    TagMeasurement meas;
    meas.tag_id = 1;
    meas.corners = corners_px;
    meas.pixel_sigma = 1.0;
    meas.timestamp = 0.1;
    meas.turret_yaw_rad = 0.0;
    
    estimator.addTagMeasurement(meas);
    
    // Add small odometry (robot didn't move)
    OdometryMeasurement odom;
    odom.dx = 0; odom.dy = 0; odom.dtheta = 0; odom.timestamp = 0.1;
    estimator.processOdometry(odom);
    
    // Update
    std::cout << "Updating...\n";
    auto result = estimator.update();
    
    std::cout << "Result Pose: x=" << result.x << ", y=" << result.y << ", theta=" << result.theta << "\n";
    std::cout << "Expected: x=0.0\n";
    
    if (std::abs(result.x) > 1.0) {
        std::cout << "FAIL: Did not converge close to 0.0 in one step.\n";
        return 1;
    } else {
        std::cout << "SUCCESS: Converged.\n";
        return 0;
    }
}
