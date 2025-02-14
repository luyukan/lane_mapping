//
// Created by yukan on 25-1-23.
//
#include "visualization/pangolin_drawer.h"

namespace mono_lane_mapping {
PangolinDrawer& PangolinDrawer::GetInstance() {
  static PangolinDrawer instance;
  return instance;
}
PangolinDrawer::PangolinDrawer() {}

void PangolinDrawer::Draw() {
  auto& variables = VisualizationVariable::GetInstance();
  const auto& tracking_lanes = variables.GetTrackingLanes();
  const auto& latest_pose = variables.GetVehiclePose();
  draw_tracking_lanes(tracking_lanes);
  draw_vehicle(latest_pose);
}

void PangolinDrawer::draw_tracking_lanes(
    const std::map<int, LaneLandmark::Ptr>& tracking_lanes) {}
void PangolinDrawer::draw_vehicle(const Odometry& pose) {}
}  // namespace mono_lane_mapping