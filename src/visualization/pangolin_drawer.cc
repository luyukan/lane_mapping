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
  if (shift_set_ == false) {
    shift_.x() = latest_pose.twb.x();
    shift_.y() = latest_pose.twb.y();
  }
  draw_vehicle(latest_pose);
  draw_tracking_lanes(tracking_lanes);
}

void PangolinDrawer::draw_tracking_lanes(
    const std::map<int, LaneLandmark::Ptr>& tracking_lanes) {
  for (auto it = tracking_lanes.begin(); it != tracking_lanes.end(); ++it) {
    glBegin(GL_LINES_ADJACENCY);
    auto lane_points = it->second->GetLanePoints();
    for (size_t i = 0; i < lane_points.size(); ++i) {
      glVertex3d(lane_points.at(i).position.x() - shift_.x(),
                 lane_points.at(i).position.y() - shift_.y(),
                 lane_points.at(i).position.z());
      std::cout << lane_points.at(i).position.x() - shift_.x() << std::endl;
    }
    glEnd();
  }
}
void PangolinDrawer::draw_vehicle(const Odometry& pose) {}
}  // namespace mono_lane_mapping