#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Eigen>

#include <flann/flann.hpp>

#include "type_define.h"
#include "system_param.h"

#include "sliding_window.h"
namespace mono_lane_mapping
{
  class LaneTracker {
    public:
      typedef std::shared_ptr<LaneTracker> Ptr;
      LaneTracker();
      static LaneTracker &GetInstance();
      void Init();
      std::vector<MatchResult> TrackWithMap(const FrameObservation &frame_observation, const Odometry &pose);
      std::vector<MatchResult> AssociateDetectionWithLast();
    private:
      double get_matching_thresh();
      double yaw_std_{5.0};
      double trans_std_{5.0};
      double lane_width_{3.5};

  };
} // namespace mono_lane_mapping
