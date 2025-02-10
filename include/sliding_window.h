#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <memory>

#include "system_param.h"
#include "type_define.h"

namespace mono_lane_mapping {
class SlidingWindow {
 public:
  typedef std::shared_ptr<SlidingWindow> Ptr;
  static SlidingWindow &GetInstance();
  SlidingWindow();
  void Init();
  void UpdateWindowStatus(const FrameObservation &frame_observation,
                          const Odometry &pose,
                          const std::vector<MatchResult> &matching_res);

 private:
  void slide_window();
  int sliding_window_size_{5};
  std::vector<WindowInfo> sl_win_;
  std::map<int, WindowLandMarkTrackInfo> sl_win_lm_info_;
};
}  // namespace mono_lane_mapping
