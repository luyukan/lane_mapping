#include "lane_tracker.h"

namespace mono_lane_mapping {

LaneTracker &LaneTracker::GetInstance() {
  static LaneTracker instance;
  return instance;
}
LaneTracker::LaneTracker() {}

void LaneTracker::Init() {
  
}

std::vector<MatchResult> LaneTracker::AssociateDetectionWithLast() {
  std::vector<MatchResult> association;
  return association;
}


}  // namespace mono_lane_mapping
