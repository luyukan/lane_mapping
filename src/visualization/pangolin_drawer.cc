//
// Created by yukan on 25-1-23.
//
#include "visualization/pangolin_drawer.h"

namespace mono_lane_mapping {
PangolinDrawer &PangolinDrawer::GetInstance() {
  static PangolinDrawer instance;
  return instance;
}
PangolinDrawer::PangolinDrawer() {}
}  // namespace mono_lane_mapping