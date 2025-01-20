#pragma once

#include <Eigen/Eigen>

namespace mono_lane_mapping {
struct Odometry {
  Eigen::Vector3d twb;
  Eigen::Quaterniond qwb;
  int64_t timestamp;
};

struct LanePoint {
  Eigen::Vector3d point_wcs;
  double visibility;
};

struct LaneObservation {
  std::vector<LanePoint> lane_points;
  int64_t local_id;
  int64_t track_id;
};

struct FrameObservation {
  std::vector<LaneObservation> lane_observations;
  int64_t timestamp;
};

}  // namespace mono_lane_mapping