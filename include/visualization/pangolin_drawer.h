//
// Created by yukan on 25-1-23.
//
#pragma once

#include <iostream>
#include <memory>
#include <random>
#include <pangolin/pangolin.h>

#include <Eigen/Eigen>
#include "visualization/visualization_variable.h"

const double Vehicle_height = 1.2;
const double Vehicle_width = 1.2;
const double Vehicle_forward = 3.718;
const double Vehicle_backward = -0.921;

namespace mono_lane_mapping {
class PangolinDrawer {
 public:
  typedef std::shared_ptr<PangolinDrawer> Ptr;
  static PangolinDrawer &GetInstance();
  PangolinDrawer();
  void Draw();
  void Init();

 private:
  void draw_tracking_lanes(
      const std::map<int, LaneLandmark> &tracking_lanes);
  void draw_vehicle(const Odometry &pose);
  void draw_coordinate_system();
  void init_color_map();
  Eigen::Vector2d shift_;
  bool shift_set_{false};
  std::map<int, Eigen::Vector3d> color_map_;
};
}  // namespace mono_lane_mapping