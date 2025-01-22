#pragma once
#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <set>
#include "type_define.h"

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
  uint8_t GetCategory();
  void SetCategory(uint8_t category);

 private:
  void curve_fitting(const std::vector<LanePoint> &lane_points);
  void init_control_points(const std::vector<LanePoint> &lane_points);
  uint64_t id_{0};
  uint8_t category_{0};
  Eigen::VectorXd cubic_polynomials_xy_;
  Eigen::VectorXd cubic_polynomials_xz_;
  Eigen::Matrix3d poly_rotation_;
  std::vector<LanePoint> control_points_;
};
}  // namespace mono_lane_mapping
