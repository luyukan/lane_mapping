#include "lane_landmark.h"

namespace mono_lane_mapping {
LaneLandmark::LaneLandmark() {}

uint64_t LaneLandmark::GetId() { return id_; }

void LaneLandmark::SetId(uint64_t id) { id_ = id; }

void LaneLandmark::InitCtrlPointsWithLaneObservation(
    const LaneObservation &lane_observation) {

}

void LaneLandmark::SetCategory(uint8_t category) {
  category_ = category;
}

uint8_t LaneLandmark::GetCategory() {
  return category_;
}
}  // namespace mono_lane_mapping
