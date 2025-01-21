#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Eigen>


#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include "sliding_window.h"
#include "gtsam_factor.h"
#include "map_graph.h"



namespace mono_lane_mapping {
class GTSAMSolver {
 public:
  typedef std::shared_ptr<GTSAMSolver> Ptr;
  static GTSAMSolver &GetInstance();
  GTSAMSolver();
  void PerformBundleAjustment();
  void Init();

 private:
  void reset();
  // gtsam::NonlinearFactorGraph graph_;
  // gtsam::Values initial_estimate_;
  // gtsam::ISAM2 isam_;
};
}  // namespace mono_lane_mapping
