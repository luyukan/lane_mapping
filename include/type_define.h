#pragma once

#include <Eigen/Eigen>

namespace mono_lane_mapping {
struct Odometry {
  Eigen::Vector3d twb;
  Eigen::Quaterniond qwb;
  long long timestamp;
};

struct LanePoint {
  Eigen::Vector3d position;
  double visibility;
};

struct LaneObservation {
  std::vector<LanePoint> lane_points;
  int local_id;
  int category;
  int map_id{-1};
};

struct FrameObservation {
  std::vector<LaneObservation> lane_observations;
  long long timestamp;
};

struct MatchResult {
  int queryIdx;  // local observation id
  int trainIdx;  // landmark id
};

struct PreProcessParameter {
  double downsample_distance{0.0};
  int observation_pts_num_min{10};
};

struct LaneMappingParameter {
  double ctrl_points_chord{0.0};
  double candidate_angle_thresh{0.0};
  int sliding_window_size{5};
  Eigen::Vector3d ctrl_noise;
  double keyframe_translation_thresh{5.0};
};

struct LaneAssoParameter {
  double yaw_std{5.0};          // deg
  double translation_std{5.0};  // meter
  double lane_width{3.5};       // meter
  double min_match_ratio{0.5};
};

struct WindowInfo {
  FrameObservation frame_observation;
  Odometry pose;
};

struct WindowLandMarkTrackInfo {
  int lm_id;
  int window_start_id;
  int window_end_id;
};

struct CubicPolyLine {
  Eigen::Matrix3d poly_rotation;
  Eigen::VectorXd cubic_polynomials_xy;
  Eigen::VectorXd cubic_polynomials_xz;
};

struct LandMarkTrackingInfo {
  int map_id;
  int tracking_times{0};
  std::vector<int> window_pose_id;
  std::vector<LaneObservation> window_observations;
};

}  // namespace mono_lane_mapping