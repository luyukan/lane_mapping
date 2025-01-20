//
// Created by yukan on 25-1-20.
//
#include "ceres_optimizer.h"

namespace mono_lane_mapping {
CeresOptimizer::CeresOptimizer() {}

CeresOptimizer &CeresOptimizer::GetInstance() {
  static CeresOptimizer instance;
  return instance;
}
};  // namespace mono_lane_mapping
