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
  if (sl_win_.size() <= sliding_window_size_ - 1) {
    sl_win_.push_back(window_info);
  } else {
    slide_window();
    sl_win_.back() = window_info;
  }
}
}  // namespace mono_lane_mapping
