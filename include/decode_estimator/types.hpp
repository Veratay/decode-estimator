#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace decode {

/// Odometry measurement in robot body frame
struct OdometryMeasurement {
    double dx;        ///< Forward displacement (meters)
    double dy;        ///< Lateral displacement (meters)
    double dtheta;    ///< Rotation (radians)
    double timestamp; ///< Timestamp (seconds)
};

/// AprilTag detection measurement (pixel coordinates)
struct TagMeasurement {
    int32_t tag_id; ///< AprilTag ID
    
    /// Detected corners in pixel coordinates (u, v)
    /// Order matters: usually Top-Left, Top-Right, Bottom-Right, Bottom-Left
    std::vector<std::pair<double, double>> corners;
    
    double pixel_sigma; ///< Uncertainty in pixel coordinates (pixels)
    double timestamp;   ///< Timestamp (seconds)
    double turret_yaw_rad = 0.0; ///< Known turret yaw at time of measurement
};

/// Robot pose estimate
struct PoseEstimate {
    double x;         ///< X position in field coordinates (meters)
    double y;         ///< Y position in field coordinates (meters)
    double theta;     ///< Heading in field coordinates (radians)
    double timestamp; ///< Timestamp (seconds)

    /// 3x3 covariance matrix (row-major: [xx, xy, xtheta, yx, yy, ytheta, thetax, thetay, thetatheta])
    std::array<double, 9> covariance;
    bool has_covariance; ///< Whether covariance is valid
};

/// AprilTag landmark definition
struct Landmark {
    int32_t id; ///< AprilTag ID
    
    // Pose of the tag center in World Frame
    double x;     ///< X position (meters)
    double y;     ///< Y position (meters)
    double z;     ///< Z position (meters)
    
    double roll;  ///< Rotation around X (radians)
    double pitch; ///< Rotation around Y (radians)
    double yaw;   ///< Rotation around Z (radians)
    
    double size;  ///< Tag side length (meters)
};

} // namespace decode
