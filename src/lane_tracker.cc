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
  const auto &lane_mapping_parameter =
      SystemParam::GetInstance().GetLaneMappingParameters();
  yaw_std_ = lane_asso_parameters.yaw_std;
  trans_std_ = lane_asso_parameters.translation_std;
  lane_width_ = lane_asso_parameters.lane_width;
  xyz_std_ = lane_mapping_parameter.ctrl_noise[0];
  min_match_ratio_ = lane_asso_parameters.min_match_ratio;
}

std::vector<double> LaneTracker::get_matching_thresh(
    const LaneObservation &observation) {
  std::vector<double> thresh;
  for (size_t i = 0; i < observation.lane_points.size(); ++i) {
    double range = observation.lane_points.at(i).position.norm();
    double yaw_th = 2.0 * yaw_std_;
    double upper_th_R = 2.0 * range * std::sin(yaw_th / 180.0 * M_PI / 2);
    double upper_th_t = trans_std_ * 2.0;
    double uppper_th_xyz = xyz_std_ * 2.0;
    double uppper_th = upper_th_R + upper_th_t + uppper_th_xyz;
    uppper_th = std::max(uppper_th, 1.0);
    thresh.push_back(uppper_th);
  }

  return thresh;
}

std::vector<MatchResult> LaneTracker::TrackWithMap(
    const FrameObservation &frame_observation, const Odometry &pose) {
  std::vector<MatchResult> matching_res;
  const auto &w = SlidingWindow::GetInstance();
  std::set<int> candidate_lm_id = w.GetCurrentTrackingLandmarkId();
  const auto &map_graph = MapGraph::GetInstance();
  std::cout << "Candidate Landmark: \n";
  for (auto it = candidate_lm_id.begin(); it != candidate_lm_id.end(); ++it) {
    std::cout << "----- " << *it << std::endl;
  } 
  std::map<int, KDTree::Ptr> lm_trees =
      map_graph.GetLandmarkTrees(candidate_lm_id);
  Eigen::Matrix3d Rwb = pose.qwb.toRotationMatrix();
  Eigen::Vector3d twb = pose.twb;

  std::vector<int> unassigned_ref_idx, unassigned_new_idx;
  std::vector<std::vector<double>> cost_vector(
      frame_observation.lane_observations.size());

  const double max_cost = 10000.0;
#if 0
  for (size_t i = 0; i < frame_observation.lane_observations.size(); ++i) {
    std::vector<double> distance_thresh =
        get_matching_thresh(frame_observation.lane_observations.at(i));
    cost_vector.at(i).resize(candidate_lm_id.size());
    std::vector<Eigen::VectorXd> points_vector;
    for (size_t j = 0; j < frame_observation.lane_observations.size(); ++i) {
      Eigen::Vector3d pt_w = Rwb * frame_observation.lane_observations.at(j)
                                       .lane_points.at(j)
                                       .position +
                             twb;
      points_vector.push_back(pt_w);
    }
    int j = 0;
    for (auto it_tree = lm_trees.begin(); it_tree != lm_trees.end();
         ++it_tree) {
      std::vector<int> matching_id;
      std::vector<double> matching_distance;
      it_tree->second->Query(points_vector, matching_distance, matching_id);
      int valid_match_num = 0;
      double dist_match = 0.0;
      double dist_match_full = 0.0;
      for (size_t j = 0; j < matching_distance.size(); ++j) {
        if (matching_distance.at(j) < distance_thresh.at(j)) {
          dist_match += matching_distance.at(j);
          ++valid_match_num;
        }
        dist_match_full += matching_distance.at(j);
      }
      double tmp0 = std::sqrt(static_cast<double>(matching_id.size()) /
                              static_cast<double>(valid_match_num));
      double score = dist_match / static_cast<double>(valid_match_num) * tmp0;
      double tmp1 = std::sqrt(1.0 / min_match_ratio_);
      double ideal_score =
          dist_match_full / static_cast<double>(matching_id.size()) * tmp1;
      double cost = score < ideal_score ? score : max_cost;
      cost_vector.at(i).at(j) = cost;
      ++j;
    }
  }

  std::vector<int> correspondences;
  std::fill(correspondences.begin(), correspondences.end(), -1);
  HungarianOptimizer hungarian_optimizer(cost_vector);
  hungarian_optimizer.Minimize(&unassigned_ref_idx, &unassigned_new_idx);
#endif
  for (size_t i = 0; i < frame_observation.lane_observations.size(); ++i) {
    MatchResult default_res;
    default_res.queryIdx = i;
    default_res.trainIdx = -1;
    matching_res.push_back(default_res);
  }

#if 0
    for (size_t i = 0; i < unassigned_ref_idx.size(); ++i) {
      int idx_mea = unassigned_ref_idx.at(i);
      auto it_map_id = candidate_lm_id.begin();
      std::advance(it_map_id, unassigned_new_idx.at(i));
      int idx_map = *it_map_id;
      if (cost_vector[idx_mea][idx_map] < max_cost) {
        matching_res.at(idx_mea).trainIdx = idx_map;
      }
    }
#endif
  std::cout << "matching info:\n";
  for (size_t i = 0; i < matching_res.size(); ++i) {
    std::cout << matching_res.at(i).queryIdx << " "
              << matching_res.at(i).trainIdx << std::endl;
  }
  return matching_res;
}

std::vector<MatchResult> LaneTracker::AssociateDetectionWithLast() {
  std::vector<MatchResult> association;
  return association;
}

}  // namespace mono_lane_mapping
