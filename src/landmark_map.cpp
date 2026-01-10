#include "decode_estimator/landmark_map.hpp"

#include <gtsam/geometry/Rot3.h>

namespace decode {

void LandmarkMap::addLandmark(int32_t id, double x, double y, double z,
                              double roll, double pitch, double yaw, double size) {
    Landmark lm;
    lm.id = id;
    lm.x = x;
    lm.y = y;
    lm.z = z;
    lm.roll = roll;
    lm.pitch = pitch;
    lm.yaw = yaw;
    lm.size = size;
    landmarks_[id] = lm;
}

void LandmarkMap::addLandmark(const Landmark& landmark) {
    landmarks_[landmark.id] = landmark;
}

void LandmarkMap::loadFromVector(const std::vector<Landmark>& landmarks) {
    landmarks_.clear();
    landmarks_.reserve(landmarks.size());
    for (const auto& lm : landmarks) {
        addLandmark(lm);
    }
}

std::optional<Landmark> LandmarkMap::getLandmark(int32_t id) const {
    auto it = landmarks_.find(id);
    if (it != landmarks_.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::optional<gtsam::Pose3> LandmarkMap::getLandmarkPose(int32_t id) const {
    auto it = landmarks_.find(id);
    if (it != landmarks_.end()) {
        const auto& lm = it->second;
        return gtsam::Pose3(
            gtsam::Rot3::Ypr(lm.yaw, lm.pitch, lm.roll),
            gtsam::Point3(lm.x, lm.y, lm.z)
        );
    }
    return std::nullopt;
}

bool LandmarkMap::hasLandmark(int32_t id) const {
    return landmarks_.find(id) != landmarks_.end();
}

const std::unordered_map<int32_t, Landmark>& LandmarkMap::getAllLandmarks() const {
    return landmarks_;
}

size_t LandmarkMap::size() const {
    return landmarks_.size();
}

void LandmarkMap::clear() {
    landmarks_.clear();
}

} // namespace decode