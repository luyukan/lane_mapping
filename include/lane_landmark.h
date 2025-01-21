#pragma once
#include <memory>
#include <iostream>
#include <Eigen/Eigen>

#include "type_define.h"

namespace mono_lane_mapping
{
  class LaneLandmark {
    public:
      typedef std::shared_ptr<LaneLandmark> Ptr;
      LaneLandmark();
      uint64_t GetId();
      void SetId(uint64_t id);
      void InitCtrlPointsWithLaneObservation(const LaneObservation &lane_observation);
      uint8_t GetCategory();
      void SetCategory(uint8_t category);
    private:
      uint64_t id_{0};
      uint8_t category_{0};
  };
} // namespace mono_lane_mapping

