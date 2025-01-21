#pragma once
#include <flann/flann.h>
#include <float.h>

#include <Eigen/Eigen>
#include <deque>
#include <iostream>
#include <memory>
#include <set>

#include "catmull_rom_spline.h"
#include "catmull_rom_spline_list.h"
#include "kd_tree.h"
#include "system_param.h"
#include "type_define.h"
#include "utils.h"

namespace mono_lane_mapping {
class LaneLandmark {
 public:
  typedef std::shared_ptr<LaneLandmark> Ptr;
  LaneLandmark();
  int GetId();
  void SetId(int id);
  bool InitCtrlPointsWithLaneObservation(
      const LaneObservation &lane_observation, const Odometry &pose);
  bool UpdateCtrlPointsWithLaneObservation(
      const LaneObservation &lane_observation, const Odometry &pose);
  uint8_t GetCategory();
  void SetCategory(uint8_t category);
  // update points(from catmull splin) and corresponding kdtree
  void CatMullSmooth();
  KDTree::Ptr GetSearchTree();
  std::vector<LanePoint> GetLanePoints() const;
  std::deque<LanePoint> GetControlPoints() const;
  KDTree::Ptr GetCtrlSearchTree();
  LanePoint GetControlPoint(const int id);
  bool FindFootPoint(const LanePoint &measurement, const Odometry &pose,
                     LaneLandmark::Ptr lm, std::vector<int> &ctrl_pts_id,
                    double &u);
  void SetControlPosition(int id, const Eigen::Vector3d &position);
 private:
  std::vector<LanePoint> update_lane_points(
      const std::vector<LanePoint> &lane_points,
      const std::set<int> &no_assigned);
  CubicPolyLine curve_fitting(const std::vector<LanePoint> &lane_points);
  // funcion used to initialize or update control points
  void get_skeleton_points(const std::vector<LanePoint> &lane_points,
                           const CubicPolyLine &cubic_lane,
                           const LanePoint &initial_point, bool initialization);
  std::vector<LanePoint> get_candidate_points(
      const std::vector<LanePoint> &lane_points);
  LanePoint get_query_point(const LanePoint &start_point,
                            const LanePoint &end_point,
                            const CubicPolyLine &cublic_lane);
  LanePoint get_next_node(const LanePoint &query_point,
                          const LanePoint &center_point, double radius,
                          const CubicPolyLine &cubic_lane);
  const LanePoint &get_latest_control_point();
  const LanePoint &get_oldest_control_point();
  const LanePoint &get_second_latest_control_point();
  const LanePoint &get_second_oldest_control_point();

  const LanePoint &get_latest_control_point_by_id(const int &id);
  const LanePoint &get_oldest_control_point_by_id(const int &id);

  void append_ctrl_tail(const LanePoint &ctrl_pt);
  void append_ctrl_head(const LanePoint &ctrl_pt);
  void revert_ctrl_points();
  void find_border_point(const std::vector<LanePoint> &candidate_points,
                         const LanePoint &query_point,
                         std::set<int> &no_assigned, LanePoint &inner_border,
                         LanePoint &outer_border, bool &outer_border_found);
  Eigen::Vector3d get_nearest_on_circle(const Eigen::Vector3d &query,
                                        const Eigen::Vector3d &center,
                                        const double radius);
  void refresh_lane_points_with_ctrl_points();
  void refresh_kd_tree();
  void refresh_ctrl_kd_tree();
  void padding_control_points();
  void perform_quality_examine();
  // inner_border 是距离query_point小于ctrl_points_chord的最远的点
  // outer_border 是距离query_point大于ctrl_points_chord的最近的点
  // no_assigned是距离query_point大于ctrl_points_chord的点的id
  const LanePoint &get_control_point(const size_t id);

  bool parameterization(const LanePoint &point,
                        const std::vector<LanePoint> &ctrl_points4,
                        double &error, double &u);
  int id_{0};
  int category_{0};
  Eigen::VectorXd cubic_polynomials_xy_;
  Eigen::VectorXd cubic_polynomials_xz_;
  Eigen::Matrix3d poly_rotation_;
  std::deque<LanePoint> control_points_;
  std::vector<LanePoint> lane_points_;

  double candidate_angle_thresh_{0.0};
  double ctrl_points_chord_{0.0};

  CatmullRomSplineList::Ptr curve_line_;
  double tau_{0.5};

  KDTree::Ptr kd_tree_;

  KDTree::Ptr ctrl_kd_tree_;

  double downsample_distance_{0.5};
};
}  // namespace mono_lane_mapping
