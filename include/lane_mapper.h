#pragma once

#include <iostream>
#include <memory>

#include "type_define.h"
#include "lane_preprocessor.h"

namespace mono_lane_mapping {
class LaneMapper {
 public:
  typedef std::shared_ptr<LaneMapper> Ptr;
  static LaneMapper &GetInstance();
  LaneMapper();
  void Init(const std::string &config_file);
  void InputSyncData(const Odometry &pose,
                     const FrameObservation &frame_observation);

 private:
  void preprocess_lane_points(const FrameObservation &frame_observation,
                              FrameObservation &cur_frame_observation);
};

}  // namespace mono_lane_mapping
