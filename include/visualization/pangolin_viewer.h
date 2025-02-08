//
// Created by yukan on 25-1-23.
//
#pragma once
#include <memory>
#include <iostream>
#include <thread>
#include <Eigen/Eigen>
#include <pangolin/pangolin.h>

namespace mono_lane_mapping
{
class PangolinViewer {
 public:
  std::shared_ptr<PangolinViewer> Ptr;
  static PangolinViewer &GetInstance();
  PangolinViewer();
  void Init();
  void Start();
 private:
  void run();
};
} // namespace mono_lane_mapping