#pragma once

#include "types.hpp"

#include <gtsam/geometry/Point2.h>

#include <optional>
#include <unordered_map>
#include <vector>

namespace decode {

/// Storage for known AprilTag landmark positions
class LandmarkMap {
public:
    LandmarkMap() = default;

    /// Add a landmark by ID and position
    void addLandmark(int32_t id, double x, double y);

    /// Add a landmark from struct
    void addLandmark(const Landmark& landmark);

    /// Load landmarks from a vector
    void loadFromVector(const std::vector<Landmark>& landmarks);

    /// Get landmark position by ID (returns nullopt if not found)
    std::optional<gtsam::Point2> getLandmark(int32_t id) const;

    /// Check if landmark exists
    bool hasLandmark(int32_t id) const;

    /// Get all landmarks (for visualization)
    const std::unordered_map<int32_t, gtsam::Point2>& getAllLandmarks() const;

    /// Get number of landmarks
    size_t size() const;

    /// Clear all landmarks
    void clear();

private:
    std::unordered_map<int32_t, gtsam::Point2> landmarks_;
};

} // namespace decode
