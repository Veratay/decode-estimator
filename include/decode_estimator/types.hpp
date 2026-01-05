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

/// AprilTag bearing measurement in robot frame
struct BearingMeasurement {
    int32_t tag_id;          ///< AprilTag ID
    double bearing_rad;      ///< Bearing angle in robot frame (radians), 0 = forward
    double uncertainty_rad;  ///< 1-sigma uncertainty (radians)
    double timestamp;        ///< Timestamp (seconds)
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

/// AprilTag landmark position
struct Landmark {
    int32_t id; ///< AprilTag ID
    double x;   ///< X position in field coordinates (meters)
    double y;   ///< Y position in field coordinates (meters)
};

} // namespace decode
