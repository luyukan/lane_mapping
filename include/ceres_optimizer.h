#pragma once

#include <memory>
#include <iostream>

#include <ceres/ceres.h>

namespace mono_lane_mapping {
  class CeresOptimizer {
   public:
    typedef std::shared_ptr<CeresOptimizer> Ptr;
    static CeresOptimizer &GetInstance();
    CeresOptimizer();
   private:
  };
};  // namespace mono_lane_mapping