#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Eigen>
#include <iostream>
#include <memory>

namespace mono_lane_mapping {
class GTSAMSolver {
 public:
  typedef std::shared_ptr<GTSAMSolver> Ptr;
  static GTSAMSolver &GetInstance();
  GTSAMSolver();
  void PerformBundleAjustment();
  void Init();

 private:
  gtsam::NonlinearFactorGraph graph_;
};
}  // namespace mono_lane_mapping
