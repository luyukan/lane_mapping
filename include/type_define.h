#pragma once

#include <Eigen/Eigen>

namespace mono_lane_mapping {
struct Odometry {
  Eigen::Vector3d twb;
  Eigen::Quaterniond qwb;
  uint64_t timestamp;
};

struct LanePoint {
  Eigen::Vector3d position;
  double visibility;
};

struct LaneObservation {
  std::vector<LanePoint> lane_points;
  uint32_t local_id;
  uint8_t category;
};

struct FrameObservation {
  std::vector<LaneObservation> lane_observations;
  int64_t timestamp;
};

struct MatchResult {
  int32_t queryIdx;
  int32_t trainIdx;
};

struct PreProcessParameter {
  double downsample_distance{0.0};
};

struct LaneMappingParameter {
  double ctrl_points_chord{0.0};
  double candidate_angle_thresh{0.0};
};


}  // namespace mono_lane_mapping