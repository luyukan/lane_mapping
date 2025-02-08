#include "lane_landmark.h"

namespace mono_lane_mapping {
LaneLandmark::LaneLandmark() {
  const auto &lane_mapping_paramter = SystemParam::GetInstance().GetLaneMappingParameters();
  candidate_angle_thresh_ = lane_mapping_paramter.candidate_angle_thresh;
  ctrl_points_chord_ = lane_mapping_paramter.ctrl_points_chord;
}

uint64_t LaneLandmark::GetId() { return id_; }

void LaneLandmark::SetId(uint64_t id) { id_ = id; }

bool LaneLandmark::InitCtrlPointsWithLaneObservation(
    const LaneObservation &lane_observation, const Odometry &pose) {
  Eigen::Vector3d twb = pose.twb;
  Eigen::Matrix3d Rwb = pose.qwb.toRotationMatrix();

  std::vector<LanePoint> lane_points_world;
  for (size_t i = 0; i < lane_observation.lane_points.size(); ++i) {
    Eigen::Vector3d pt_w =
        Rwb * lane_observation.lane_points.at(i).position + twb;
    LanePoint lane_point;
    lane_point.position = pt_w;
    lane_points_world.push_back(lane_point);
  }

  curve_fitting(lane_points_world);
  init_control_points(lane_points_world);

}

void LaneLandmark::init_control_points(const std::vector<LanePoint> &lane_points) {
  std::vector<LanePoint> skeleton_points;
  LanePoint initial_point = lane_points.at(0);
  control_points_.push_back(initial_point);
  extend_control_points(lane_points);

}

void LaneLandmark::find_border_point(const std::vector<LanePoint> &candidate_points,
                                     const LanePoint &query_point,
                                     std::set<int> &no_assigned,
                                     LanePoint &inner_border,
                                     LanePoint &outer_border,
                                     bool &outer_border_found) {
  double max_distance = .0;
  double min_distance = DBL_MAX;
  for (size_t i = 0; i < candidate_points.size(); ++i) {
    double distance = (candidate_points.at(i).position - query_point.position).norm();
    if (distance < ctrl_points_chord_) {
      auto it = no_assigned.find(i);
      no_assigned.erase(it);
      if (distance > max_distance) {
        max_distance = distance;
        inner_border = candidate_points.at(i);
      }
      else {
        if (distance < min_distance) {
          min_distance = distance;
          outer_border = candidate_points.at(i);
          outer_border_found = true;
        }
      }
    }
  }
}

std::vector<LanePoint> LaneLandmark::get_candidate_points(const std::vector<LanePoint> &lane_points) {
  if (control_points_.size() < 2) {
    return lane_points;
  } else {
    std::vector<LanePoint> candidate_points;
    size_t num_ctrl_point = control_points_.size();

    const auto &ctrl_pt_0 = get_control_point(0); // first ctrl
    const auto &ctrl_pt_1 = get_control_point(1); // second ctrl

    const auto &ctrl_pt_reverse_0 = get_control_point(num_ctrl_point - 1); // last ctrl
    const auto &ctrl_pt_reverse_1 = get_control_point(num_ctrl_point - 2); // second to last ctrl

    Eigen::Vector3d normal_a = ctrl_pt_0.position - ctrl_pt_1.position;
    normal_a.normalize();
    Eigen::Vector3d normal_b = ctrl_pt_reverse_0.position - ctrl_pt_reverse_1.position;
    normal_b.normalize();

    for (size_t i = 0; i < lane_points.size(); ++i) {
      Eigen::Vector3d d_a = (lane_points.at(i).position - ctrl_pt_0.position).normalized();
      Eigen::Vector3d d_b = (lane_points.at(i).position - ctrl_pt_reverse_0.position).normalized();
      double cos_a = d_a.dot(normal_a);
      double cos_b = d_b.dot(normal_b);
      double cos_thresh = std::cos(Deg2Rad(candidate_angle_thresh_));
      if (cos_a > cos_thresh || cos_b > cos_thresh) {
        candidate_points.push_back(lane_points.at(i));
      }

    }
  }
}

const LanePoint &LaneLandmark::get_control_point(const size_t id) {
  return control_points_.at(id);
}

void LaneLandmark::extend_control_points(const std::vector<LanePoint> &lane_points) {
  // fairly simple implementation

  std::vector<LanePoint> candidate_points = get_candidate_points(lane_points);
  LanePoint inner_border, outer_border; //
  bool outer_border_found{false};
  LanePoint initial_point = lane_points.at(0);

  while (true) {

  }
  // initialize id set

  std::set<int> no_assigned;
  for (size_t i = 0; i < candidate_points.size(); ++i) {
    no_assigned.insert(i);
  }
  find_border_point(lane_points, initial_point, no_assigned, inner_border, outer_border, outer_border_found);
  if (outer_border_found == false) {
    int start_id = 0;
    int end_id = control_points_.size() - 1;
    double dist_head = (control_points_.at(start_id).position - inner_border.position).norm();
    double dist_tail = (control_points_.at(end_id).position - control_points_.at)
  }
  else {

  }
}

void LaneLandmark::SetCategory(uint8_t category) { category_ = category; }

uint8_t LaneLandmark::GetCategory() { return category_; }

void LaneLandmark::curve_fitting(const std::vector<LanePoint> &lane_points) {
  Eigen::MatrixXd data = ConstructDataMatrix(lane_points);
  Eigen::MatrixXd transformed_data;
  Eigen::Matrix3d data_rotation;
  Eigen::VectorXd target_axis = Eigen::Vector2d::UnitY();
  GetTransformedData(data, target_axis, transformed_data, data_rotation);

  Eigen::VectorXd poly_xy = CubicPolyFit(transformed_data.col(0), transformed_data.col(1));
  Eigen::VectorXd poly_xz = CubicPolyFit(transformed_data.col(0), transformed_data.col(2));

  poly_rotation_ = data_rotation;
  cubic_polynomials_xy_ = poly_xy;
  cubic_polynomials_xz_ = poly_xz;
}

}  // namespace mono_lane_mapping
