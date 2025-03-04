#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <memory>

#include "lane_landmark.h"
#include "map_graph.h"
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
                          const Odometry &pose);
  std::set<int> GetCurrentTrackingLandmarkId() const;
  std::map<int, LaneLandmark::Ptr> GetCurrentTrackingLandmarks() const;
  Odometry GetLatestPose();
  Odometry GetWindowPose(int id);
  bool Initialized();
  int GetCurrentWindowSize();

  std::map<int, LandMarkTrackingInfo> GetLandmarkTrackingInfo();

 private:
  void slide_window();
  bool is_new_keyframe(const FrameObservation &frame_observation,
                       const Odometry &pose);
  int sliding_window_size_{5};
  std::vector<WindowInfo> sl_win_;
  std::map<int, WindowLandMarkTrackInfo> sl_win_lm_info_;
  double keyframe_translation_thresh_{5.0};
};
}  // namespace mono_lane_mapping
