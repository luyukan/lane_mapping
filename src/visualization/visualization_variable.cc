//
// Created by yukan on 25-1-23.
//
#include "visualization/visualization_variable.h"

namespace mono_lane_mapping {
VisualizationVariable &VisualizationVariable::GetInstance() {
  static VisualizationVariable instance;
  return instance;
}
VisualizationVariable::VisualizationVariable() {}

std::mutex &VisualizationVariable::GetVisualizationMutex() {
  return variable_mutex_;
}

void VisualizationVariable::UpdateVariables() {
  std::lock_guard<std::mutex> lock(variable_mutex_);
  {
    auto &sw = SlidingWindow::GetInstance();
    if (sw.Initialized()) {
      update_lanes(sw);
      pose_ = sw.GetLatestPose();
    }
  }
}

std::map<int, LaneLandmark> VisualizationVariable::GetTrackingLanes() {
  std::lock_guard<std::mutex> lock(variable_mutex_);
  return tracking_lanes_;
}

void VisualizationVariable::update_lanes(const SlidingWindow &w) {
  auto tracking_lanes = w.GetCurrentTrackingLandmarks();
  tracking_lanes_.clear();
  for (auto it = tracking_lanes.begin(); it != tracking_lanes.end(); ++it) {
    tracking_lanes_.insert(std::make_pair(it->first, *(it->second)));
  }
}

Odometry VisualizationVariable::GetVehiclePose() {
  std::lock_guard<std::mutex> lock(variable_mutex_);
  return pose_;
}
}  // namespace mono_lane_mapping