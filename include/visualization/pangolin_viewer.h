//
// Created by yukan on 25-1-23.
//
#pragma once
#include <pangolin/pangolin.h>

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <thread>

#include "map_graph.h"
#include "visualization/pangolin_drawer.h"

namespace mono_lane_mapping {
class PangolinViewer {
 public:
  typedef std::shared_ptr<PangolinViewer> Ptr;
  static PangolinViewer &GetInstance();
  PangolinViewer();
  void Init();
  void Start();

 private:
  void run();
  PangolinDrawer::Ptr pango_drawer_;
};
}  // namespace mono_lane_mapping