#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Eigen>

#include "type_define.h"

namespace mono_lane_mapping
{
  class LaneTracker {
    public:
      typedef std::shared_ptr<LaneTracker> Ptr;
      LaneTracker();
      static LaneTracker &GetInstance();
      void Init();

      std::vector<MatchResult> AssociateDetectionWithLast();
    private:
  };
} // namespace mono_lane_mapping
