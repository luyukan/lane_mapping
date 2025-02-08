#pragma once
#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <set>
#include <float.h>
#include "type_define.h"
#include "system_param.h"
#include "utils.h"

namespace mono_lane_mapping {
class LaneLandmark {
 public:
  typedef std::shared_ptr<LaneLandmark> Ptr;
  LaneLandmark();
  uint64_t GetId();
  void SetId(uint64_t id);
  bool InitCtrlPointsWithLaneObservation(
      const LaneObservation &lane_observation, const Odometry &pose);
  bool UpdateCtrlPointsWithLaneObservation();
  uint8_t GetCategory();
  void SetCategory(uint8_t category);

 private:
  void curve_fitting(const std::vector<LanePoint> &lane_points);
  void init_control_points(const std::vector<LanePoint> &lane_points);
  void extend_control_points(const std::vector<LanePoint> &lane_points);
  std::vector<LanePoint> get_candidate_points(const std::vector<LanePoint> &lane_points);
  void find_border_point(const std::vector<LanePoint> &candidate_points,
                         const LanePoint &query_point,
                         std::set<int> &no_assigned,
                         LanePoint &inner_border,
                         LanePoint &outer_border,
                         bool &outer_border_found);
  //inner_border 是距离query_point小于ctrl_points_chord的最远的点
  //outer_border 是距离query_point大于ctrl_points_chord的最近的点
  //no_assigned是距离query_point大于ctrl_points_chord的点的id
  const LanePoint &get_control_point(const size_t id);
  uint64_t id_{0};
  uint8_t category_{0};
  Eigen::VectorXd cubic_polynomials_xy_;
  Eigen::VectorXd cubic_polynomials_xz_;
  Eigen::Matrix3d poly_rotation_;
  std::vector<LanePoint> control_points_;

  double candidate_angle_thresh_{0.0};
  double ctrl_points_chord_{0.0};
};
}  // namespace mono_lane_mapping
