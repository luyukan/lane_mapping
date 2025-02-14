#pragma once

#include <iostream>
#include <memory>

#include "gtsam_solver.h"
#include "lane_preprocessor.h"
#include "lane_tracker.h"
#include "map_graph.h"
#include "sliding_window.h"
#include "system_param.h"
#include "type_define.h"


#include "visualization_variable.h"

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

  void init_map(const FrameObservation &frame_observation,
                const Odometry &pose);
  void track_with_last_frame(const FrameObservation &frame_observation,
                             const Odometry &pose);
  void track_with_map(const FrameObservation &frame_observation,
                      const Odometry &pose);
  void smooth();
  void printInformation();

  std::vector<LaneObservation> observation_sliding_window_;
  std::vector<Odometry> pose_sliding_window_;

  int8_t window_size_{7};
  bool initialized_{false};

  FrameObservation last_frame_observation_;

  double candidate_angle_thresh_{0.0};
};

}  // namespace mono_lane_mapping
