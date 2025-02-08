//
// Created by yukan on 25-1-23.
//
#pragma once
#include <memory>
#include <iostream>
#include <Eigen/Eigen>
#include <pangolin/pangolin.h>

namespace mono_lane_mapping
{
class PangolinDrawer {
 public:
  std::shared_ptr<PangolinDrawer> Ptr;
  static PangolinDrawer &GetInstance();
  PangolinDrawer();
 private:
};
} // namespace mono_lane_mapping