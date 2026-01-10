#pragma once

#include "types.hpp"

#include <gtsam/geometry/Pose3.h>

#include <optional>
#include <unordered_map>
#include <vector>

namespace decode {

/// Storage for known AprilTag landmark positions
class LandmarkMap {
public:
    LandmarkMap() = default;

    /// Add a landmark with full 3D pose and size
    void addLandmark(int32_t id, double x, double y, double z, 
                    double roll, double pitch, double yaw, double size);

    /// Add a landmark from struct
    void addLandmark(const Landmark& landmark);

    /// Load landmarks from a vector
    void loadFromVector(const std::vector<Landmark>& landmarks);

    /// Get landmark by ID (returns nullopt if not found)
    std::optional<Landmark> getLandmark(int32_t id) const;

    /// Get landmark as GTSAM Pose3
    std::optional<gtsam::Pose3> getLandmarkPose(int32_t id) const;

    /// Check if landmark exists
    bool hasLandmark(int32_t id) const;

    /// Get all landmarks
    const std::unordered_map<int32_t, Landmark>& getAllLandmarks() const;

    /// Get number of landmarks
    size_t size() const;

    /// Clear all landmarks
    void clear();

private:
    std::unordered_map<int32_t, Landmark> landmarks_;
};

} // namespace decode
