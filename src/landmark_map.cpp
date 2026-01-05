#include "decode_estimator/landmark_map.hpp"

namespace decode {

void LandmarkMap::addLandmark(int32_t id, double x, double y) {
    landmarks_[id] = gtsam::Point2(x, y);
}

void LandmarkMap::addLandmark(const Landmark& landmark) {
    addLandmark(landmark.id, landmark.x, landmark.y);
}

void LandmarkMap::loadFromVector(const std::vector<Landmark>& landmarks) {
    landmarks_.clear();
    landmarks_.reserve(landmarks.size());
    for (const auto& lm : landmarks) {
        addLandmark(lm);
    }
}

std::optional<gtsam::Point2> LandmarkMap::getLandmark(int32_t id) const {
    auto it = landmarks_.find(id);
    if (it != landmarks_.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool LandmarkMap::hasLandmark(int32_t id) const {
    return landmarks_.find(id) != landmarks_.end();
}

const std::unordered_map<int32_t, gtsam::Point2>& LandmarkMap::getAllLandmarks() const {
    return landmarks_;
}

size_t LandmarkMap::size() const {
    return landmarks_.size();
}

void LandmarkMap::clear() {
    landmarks_.clear();
}

} // namespace decode
