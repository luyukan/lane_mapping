#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Eigen>
#include <flann/flann.hpp>

#include "type_define.h"
#include "system_param.h"

#include "sliding_window.h"
#include "kd_tree.h"
#include "map_graph.h"
#include "hungarian.h"
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
      std::vector<double> get_matching_thresh(const LaneObservation &observation);
      double yaw_std_{5.0};
      double trans_std_{5.0};
      double lane_width_{3.5};
      double xyz_std_{0.5};
      double min_match_ratio_{0.5};

  };
} // namespace mono_lane_mapping
