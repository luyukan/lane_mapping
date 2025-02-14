//
// Created by yukan on 25-1-23.
//
#pragma once
#include <memory>
#include <iostream>
#include <Eigen/Eigen>
#include <pangolin/pangolin.h>

#include "visualization/visualization_variable.h"

namespace mono_lane_mapping
{
class PangolinDrawer {
 public:
  std::shared_ptr<PangolinDrawer> Ptr;
  static PangolinDrawer &GetInstance();
  PangolinDrawer();
  void Draw();
 private:
  void draw_tracking_lanes(const std::map<int, LaneLandmark::Ptr> &tracking_lanes);
  void draw_vehicle(const Odometry& pose);
};
} // namespace mono_lane_mapping