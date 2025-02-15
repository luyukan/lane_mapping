#include "sliding_window.h"

namespace mono_lane_mapping {
SlidingWindow::SlidingWindow() {}

SlidingWindow &SlidingWindow::GetInstance() {
  static SlidingWindow instance;
  return instance;
}

void SlidingWindow::Init() {
  const auto &lane_mapping_parameters =
      SystemParam::GetInstance().GetLaneMappingParameters();
  sliding_window_size_ = lane_mapping_parameters.sliding_window_size;
  keyframe_translation_thresh_ =
      lane_mapping_parameters.keyframe_translation_thresh;
}

void SlidingWindow::slide_window() {
  if (sl_win_.size() < sliding_window_size_) {
    return;
  }

  for (size_t i = 0; i < sliding_window_size_ - 1; ++i) {
    std::swap(sl_win_.at(i), sl_win_.at(i + 1));
  }
}

void SlidingWindow::UpdateWindowStatus(
    const FrameObservation &frame_observation, const Odometry &pose,
    const std::vector<MatchResult> &matching_res) {
  WindowInfo window_info;
  window_info.pose = pose;
  window_info.frame_observation = frame_observation;

  // update map_id
  for (size_t i = 0; i < window_info.frame_observation.lane_observations.size();
       ++i) {
    window_info.frame_observation.lane_observations.at(i).map_id =
        matching_res.at(i).trainIdx;
  }

  if (is_new_keyframe(frame_observation, pose)) {
    if (sl_win_.size() <= sliding_window_size_ - 1) {
      sl_win_.push_back(window_info);
    } else {
      slide_window();
      sl_win_.back() = window_info;
    }
  }
}

std::map<int, LaneLandmark::Ptr> SlidingWindow::GetCurrentTrackingLandmarks() {
  std::set<int> current_tracking_id = this->GetCurrentTrackingLandmarkId();
  std::map<int, LaneLandmark::Ptr> landmarks;
  auto &map = MapGraph::GetInstance();
  for (auto it = current_tracking_id.begin(); it != current_tracking_id.end();
       ++it) {
    int id = *it;
    landmarks.insert(std::make_pair(id, map.GetLandmark(id)));
  }
  std::cout << "Tracking Landmarks size: " << landmarks.size() << std::endl;
  return landmarks;
}

Odometry SlidingWindow::GetLatestPose() { return sl_win_.begin()->pose; }
bool SlidingWindow::Initialized() { return sl_win_.size() > 0 ? true : false; }

bool SlidingWindow::is_new_keyframe(const FrameObservation &frame_observation,
                                    const Odometry &pose) {
  bool is_keyframe{false};
  if (frame_observation.lane_observations.empty()) {
    is_keyframe = false;
  } else {
    if (sl_win_.size() == 0) {
      is_keyframe = true;
    } else {
      const auto &last_pose = sl_win_.back().pose;
      if ((pose.twb - last_pose.twb).norm() >= keyframe_translation_thresh_) {
        is_keyframe = true;
      }
    }
  }

  return is_keyframe;
}

std::set<int> SlidingWindow::GetCurrentTrackingLandmarkId() const {
  std::set<int> id_set;
  for (size_t i = 0; i < sl_win_.size(); ++i) {
    for (size_t j = 0;
         j < sl_win_.at(i).frame_observation.lane_observations.size(); ++j) {
      id_set.insert(
          sl_win_.at(i).frame_observation.lane_observations.at(j).map_id);
    }
  }
  return id_set;
}
}  // namespace mono_lane_mapping
