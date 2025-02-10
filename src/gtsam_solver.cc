#include "gtsam_solver.h"

namespace mono_lane_mapping {

GTSAMSolver &GTSAMSolver::GetInstance() {
  static GTSAMSolver instance;
  return instance;
}

GTSAMSolver::GTSAMSolver() {}

void GTSAMSolver::Init() {}

void GTSAMSolver::PerformBundleAjustment() {}
}  // namespace mono_lane_mapping