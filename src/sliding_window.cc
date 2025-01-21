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
    const FrameObservation &frame_observation, const Odometry &pose) {
  WindowInfo window_info;
  window_info.pose = pose;
  window_info.frame_observation = frame_observation;

  if (is_new_keyframe(frame_observation, pose)) {
    if (sl_win_.size() <= sliding_window_size_ - 1) {
      sl_win_.push_back(window_info);
    } else {
      slide_window();
      sl_win_.back() = window_info;
    }
  }
}

std::map<int, LandMarkTrackingInfo> SlidingWindow::GetLandmarkTrackingInfo() {
  std::map<int, LandMarkTrackingInfo> tracking_info;
  for (size_t i = 0; i < sl_win_.size(); ++i) {
    for (size_t j = 0;
         j < sl_win_.at(i).frame_observation.lane_observations.size(); ++j) {
      int map_id =
          sl_win_.at(i).frame_observation.lane_observations.at(j).map_id;
      if (!tracking_info.count(map_id)) {
        LandMarkTrackingInfo info;
        info.map_id = map_id;
        info.tracking_times = 1;
        info.window_pose_id.push_back(i);
        info.window_observations.push_back(
            sl_win_.at(i).frame_observation.lane_observations.at(j));
        tracking_info.insert({map_id, info});
      } else {
        tracking_info.at(map_id).tracking_times++;
        tracking_info.at(map_id).window_pose_id.push_back(i);
        tracking_info.at(map_id).window_observations.push_back(
            sl_win_.at(i).frame_observation.lane_observations.at(j));
      }
    }
  }

  return tracking_info;
}

std::map<int, LaneLandmark::Ptr> SlidingWindow::GetCurrentTrackingLandmarks()
    const {
  std::set<int> current_tracking_id = this->GetCurrentTrackingLandmarkId();
  std::map<int, LaneLandmark::Ptr> landmarks;
  auto &map = MapGraph::GetInstance();
  for (auto it = current_tracking_id.begin(); it != current_tracking_id.end();
       ++it) {
    int id = *it;
    landmarks.insert(std::make_pair(id, map.GetLandmark(id)));
  }
  // std::cout << "Tracking Landmarks size: " << landmarks.size() << std::endl;
  return landmarks;
}

Odometry SlidingWindow::GetLatestPose() { return sl_win_.rbegin()->pose; }

Odometry SlidingWindow::GetWindowPose(int id) {
  return sl_win_.at(id).pose;
}

int SlidingWindow::GetCurrentWindowSize() { return sl_win_.size(); }

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
