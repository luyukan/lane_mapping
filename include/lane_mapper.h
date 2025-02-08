#pragma once

#include <iostream>
#include <memory>

#include "lane_preprocessor.h"
#include "lane_tracker.h"
#include "map_graph.h"
#include "type_define.h"
#include "system_param.h"

namespace mono_lane_mapping {
class LaneMapper {
 public:
  typedef std::shared_ptr<LaneMapper> Ptr;
  static LaneMapper &GetInstance();
  LaneMapper();
  void Init();
  void InputSyncData(const Odometry &pose,
                     const FrameObservation &frame_observation);

 private:
  void preprocess_lane_points(const FrameObservation &frame_observation,
                              FrameObservation &cur_frame_observation);

  void init_map_graph(const FrameObservation &frame_observation, const Odometry &pose);
  void printSlogan();

  std::vector<LaneObservation> observation_sliding_window_;
  std::vector<Odometry> pose_sliding_window_;

  int8_t window_size_{7};
  bool initialized_{false};

  FrameObservation last_frame_observation_;

  double candidate_angle_thresh_{0.0};

};

}  // namespace mono_lane_mapping
