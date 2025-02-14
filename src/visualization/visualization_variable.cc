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
  auto &sw = SlidingWindow::GetInstance();
  tracking_lanes_ = sw.GetCurrentTrackingLandmarks();
  pose_ = sw.GetLatestPose();
}

std::map<int, LaneLandmark::Ptr> VisualizationVariable::GetTrackingLanes() {
  return tracking_lanes_;
}
Odometry VisualizationVariable::GetVehiclePose() {
  return pose_;
}
}  // namespace mono_lane_mapping