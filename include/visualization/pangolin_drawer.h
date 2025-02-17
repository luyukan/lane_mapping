//
// Created by yukan on 25-1-23.
//
#pragma once
#include <pangolin/pangolin.h>

#include <Eigen/Eigen>
#include <iostream>
#include <memory>

#include "visualization/visualization_variable.h"

namespace mono_lane_mapping {
class PangolinDrawer {
 public:
  typedef std::shared_ptr<PangolinDrawer> Ptr;
  static PangolinDrawer &GetInstance();
  PangolinDrawer();
  void Draw();

 private:
  void draw_tracking_lanes(
      const std::map<int, LaneLandmark::Ptr> &tracking_lanes);
  void draw_vehicle(const Odometry &pose);
  Eigen::Vector2d shift_;
  bool shift_set_{false};
};
}  // namespace mono_lane_mapping