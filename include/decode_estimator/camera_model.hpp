#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>

namespace decode {

struct CameraModel {
    double fx = 1000.0;
    double fy = 1000.0;
    double cx = 320.0;
    double cy = 240.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double k3 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
};

inline bool projectPoint(const gtsam::Pose3& camera_pose,
                         const CameraModel& model,
                         const gtsam::Point3& world_point,
                         gtsam::Point2* uv) {
    gtsam::Point3 pc = camera_pose.transformTo(world_point);
    if (pc.z() <= 1e-9) {
        return false;
    }

    double x = pc.x() / pc.z();
    double y = pc.y() / pc.z();
    double r2 = x * x + y * y;
    double r4 = r2 * r2;
    double r6 = r4 * r2;

    double radial = 1.0 + model.k1 * r2 + model.k2 * r4 + model.k3 * r6;
    double x_rad = x * radial;
    double y_rad = y * radial;

    double x_tan = 2.0 * model.p1 * x * y + model.p2 * (r2 + 2.0 * x * x);
    double y_tan = model.p1 * (r2 + 2.0 * y * y) + 2.0 * model.p2 * x * y;

    double xd = x_rad + x_tan;
    double yd = y_rad + y_tan;

    double u = model.fx * xd + model.cx;
    double v = model.fy * yd + model.cy;

    *uv = gtsam::Point2(u, v);
    return true;
}

} // namespace decode
