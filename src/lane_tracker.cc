#include "lane_tracker.h"

namespace mono_lane_mapping {

LaneTracker &LaneTracker::GetInstance() {
  static LaneTracker instance;
  return instance;
}
LaneTracker::LaneTracker() {}

void LaneTracker::Init() {
  // flann::Matrix<float> flann_dataset;
  const auto &lane_asso_parameters =
      SystemParam::GetInstance().GetLaneAssoParameters();
  yaw_std_ = lane_asso_parameters.yaw_std;
  trans_std_ = lane_asso_parameters.translation_std;
  lane_width_ = lane_asso_parameters.lane_width;
}

std::vector<MatchResult> LaneTracker::TrackWithMap(
    const FrameObservation &frame_observation, const Odometry &pose) {
  std::vector<MatchResult> matching_res;
  return matching_res;
}

std::vector<MatchResult> LaneTracker::AssociateDetectionWithLast() {
  std::vector<MatchResult> association;
  return association;
}

}  // namespace mono_lane_mapping
