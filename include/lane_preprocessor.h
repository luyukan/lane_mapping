#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "type_define.h"
#include "utils.h"
#include "system_param.h"


namespace mono_lane_mapping {
class LanePreprocessor {
 public:
  typedef std::shared_ptr<LanePreprocessor> Ptr;
  static LanePreprocessor &GetInstance();
  LanePreprocessor();
  void Init();
  void DenoiseLanePoints(const FrameObservation &frame_observation,
                         FrameObservation &cur_frame_observation);

 private:
  void denoisePoints(const std::vector<LanePoint> &lane_points, std::vector<LanePoint> &denoised_lane_points);
  double downsample_distance_{0.0};
};
}  // namespace mono_lane_mapping
