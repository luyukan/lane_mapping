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

std::vector<double> LaneTracker::get_matching_thresh() {}

std::vector<MatchResult> LaneTracker::TrackWithMap(
    const FrameObservation &frame_observation, const Odometry &pose) {
  std::vector<MatchResult> matching_res;
  const auto &w = SlidingWindow::GetInstance();
  std::set<int> candidate_lm_id = w.GetCurrentTrackingLandmarkId();
  // std::vector<KDTree::Ptr> lm_trees =

  Eigen::Matrix3d Rwb = pose.qwb.toRotationMatrix();
  Eigen::Vector3d twb = pose.twb;

  for (size_t i = 0; i < frame_observation.lane_observations.size(); ++i) {
    // Eigen::MatrixXd 
  }

  return matching_res;
}

std::vector<MatchResult> LaneTracker::AssociateDetectionWithLast() {
  std::vector<MatchResult> association;
  return association;
}

}  // namespace mono_lane_mapping
