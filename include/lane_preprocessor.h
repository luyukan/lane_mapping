#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Eigen>
#include <iostream>
#include <memory>

#include "system_param.h"
#include "type_define.h"
#include "utils.h"

namespace mono_lane_mapping {
class LanePreprocessor {
 public:
  typedef std::shared_ptr<LanePreprocessor> Ptr;
  static LanePreprocessor &GetInstance();
  LanePreprocessor();
  void Init();
  void DenoiseLaneObservation(const FrameObservation &frame_observation,
                         FrameObservation &cur_frame_observation);

 private:
  void denoisePoints(const std::vector<LanePoint> &lane_points,
                     std::vector<LanePoint> &denoised_lane_points);
  double downsample_distance_{0.0};
  int observation_pts_num_min_{10};
};
}  // namespace mono_lane_mapping
