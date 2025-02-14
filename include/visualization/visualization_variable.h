//
// Created by yukan on 25-1-23.
//
#pragma once
#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <mutex>

#include "lane_mapper.h"
#include "sliding_window.h"

namespace mono_lane_mapping {
class VisualizationVariable {
 public:
  std::shared_ptr<VisualizationVariable> Ptr;
  static VisualizationVariable& GetInstance();
  VisualizationVariable();
  std::mutex& GetVisualizationMutex();
  void UpdateVariables();
  std::map<int, LaneLandmark::Ptr> GetTrackingLanes();
  Odometry GetVehiclePose();
 private:
  std::mutex variable_mutex_;

  Odometry pose_;
  std::map<int, LaneLandmark::Ptr> tracking_lanes_;

  bool pose_init_{false};
};
}  // namespace mono_lane_mapping