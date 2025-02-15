#include "lane_landmark.h"

namespace mono_lane_mapping {
LaneLandmark::LaneLandmark() {
  const auto &lane_mapping_parameter =
      SystemParam::GetInstance().GetLaneMappingParameters();
  candidate_angle_thresh_ = lane_mapping_parameter.candidate_angle_thresh;
  ctrl_points_chord_ = lane_mapping_parameter.ctrl_points_chord;

  const auto &preprocess_parameter =
      SystemParam::GetInstance().GetPreProcessParameters();
  downsample_distance_ = preprocess_parameter.downsample_distance;
}

KDTree::Ptr LaneLandmark::GetSearchTree() { return kd_tree_; }

void LaneLandmark::CatMullSmooth() {
  if (control_points_.size() >= 2) {
    if (control_points_.size() < 4) {
      padding_control_points();
    }
    Eigen::MatrixXd points_mat =
        Eigen::MatrixXd::Zero(control_points_.size(), 3);
    for (size_t i = 0; i < control_points_.size(); ++i) {
      Eigen::Vector3d pt = control_points_.at(i).position;
      points_mat.row(i) = pt.transpose();
    }

    if (curve_line_ != nullptr) {
      curve_line_.reset(new CatmullRomSplineList(points_mat, tau_));
    } else {
      curve_line_ = std::make_shared<CatmullRomSplineList>(points_mat, tau_);
    }
    referesh_lane_points_with_ctrl_points();
    referesh_kd_tree();
  }
}

int LaneLandmark::GetId() { return id_; }

void LaneLandmark::SetId(int id) { id_ = id; }

bool LaneLandmark::UpdateCtrlPointsWithLaneObservation(
    const LaneObservation &lane_observation, const Odometry &pose) {
  return true;
}

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
  const LanePoint initial_point = lane_points_world.at(0);
  get_skeleton_points(lane_points_world, initial_point, true);

  return true;
}

std::vector<LanePoint> LaneLandmark::update_lane_points(
    const std::vector<LanePoint> &lane_points,
    const std::set<int> &no_assigned) {
  std::vector<LanePoint> lane_points_updated;
  for (auto it = no_assigned.begin(); it != no_assigned.end(); ++it) {
    int id = *it;
    lane_points_updated.push_back(lane_points.at(id));
  }

  return lane_points_updated;
}

void LaneLandmark::get_skeleton_points(
    const std::vector<LanePoint> &lane_points, const LanePoint &initial_point,
    bool initial_point_provided) {
  std::vector<LanePoint> internal_lane_points;
  LanePoint internal_initial_point;
  if (!initial_point_provided) {
    internal_initial_point = internal_lane_points.at(0);
    control_points_.push_back(internal_initial_point);
  } else {
    internal_initial_point = initial_point;
  }

  while (true) {
    auto candidate_points = get_candidate_points(internal_lane_points);
    if (candidate_points.empty()) {
      return;
    }
    // initialize id set
    std::set<int> no_assigned;
    for (size_t i = 0; i < candidate_points.size(); ++i) {
      no_assigned.insert(i);
    }
    LanePoint inner_border, outer_border;  //
    bool outer_border_found{false};
    // inner_border是距离query点距离小于chord thresh的点中距离最远的点
    // outer_border是距离query点中距离大于chord thresh中距离最近的点
    find_border_point(internal_lane_points, initial_point, no_assigned,
                      inner_border, outer_border, outer_border_found);

    if (outer_border_found == false) {
      double distance_head =
          (control_points_.at(0).position - inner_border.position).norm();
      double distance_tail =
          (control_points_.at(control_points_.size() - 1).position -
           inner_border.position)
              .norm();

      if (distance_head <=
          distance_tail) {  // first control point closer to the inner border
        if (control_points_.size() >= 2) {
          inner_border =
              get_query_point(control_points_.at(1), control_points_.at(0));
        }
        LanePoint next_initial = get_next_node(
            inner_border, control_points_.at(0), ctrl_points_chord_);
        control_points_.push_back(next_initial);
        internal_initial_point = next_initial;

      } else {  // last control point closer to the inner border
        if (control_points_.size() >= 2) {
          inner_border =
              get_query_point(control_points_.at(control_points_.size() - 2),
                              control_points_.at(control_points_.size() - 1));
        }
        LanePoint next_initial = get_next_node(
            inner_border, control_points_.at(control_points_.size() - 1),
            ctrl_points_chord_);
        control_points_.push_back(next_initial);
        internal_initial_point = next_initial;
      }

    } else {
      double distance_head =
          (control_points_.at(0).position - outer_border.position).norm();
      double distance_tail =
          (control_points_.at(control_points_.size() - 1).position -
           outer_border.position)
              .norm();

      if (distance_head <= distance_tail) {
        if (control_points_.size() > 2) {
          outer_border =
              get_query_point(control_points_.at(1), control_points_.at(0));
        }
        LanePoint next_initial = get_next_node(
            outer_border, control_points_.at(0), ctrl_points_chord_);
        control_points_.push_back(next_initial);
        internal_initial_point = next_initial;

      } else {
        if (control_points_.size() > 2) {
          outer_border =
              get_query_point(control_points_.at(control_points_.size() - 2),
                              control_points_.at(control_points_.size() - 1));
        }
        LanePoint next_initial = get_next_node(
            outer_border, control_points_.at(0), ctrl_points_chord_);
        control_points_.push_back(next_initial);
        internal_initial_point = next_initial;
      }

      internal_lane_points =
          update_lane_points(internal_lane_points, no_assigned);
    }
  }
}

LanePoint LaneLandmark::get_query_point(const LanePoint &start_point,
                                        const LanePoint &end_point) {
  LanePoint point;
  return point;
}
LanePoint LaneLandmark::get_next_node(const LanePoint &query_point,
                                      const LanePoint &center_point,
                                      double radius) {
  LanePoint point;
  return point;
}
void LaneLandmark::referesh_kd_tree() {
  if (kd_tree_ == nullptr) {
    kd_tree_ = std::make_shared<KDTree>();
  }
  kd_tree_->Reset();
  Eigen::MatrixXd lane_pts_mat = Eigen::MatrixXd::Zero(lane_points_.size(), 3);
  for (size_t i = 0; i < lane_points_.size(); ++i) {
    lane_pts_mat.row(i) = lane_points_.at(i).position.transpose();
  }
  kd_tree_->ConstructTree(lane_pts_mat);
}

void LaneLandmark::padding_control_points() {
  size_t num_ctrl = control_points_.size();
  if (num_ctrl == 3) {
    LanePoint last_ctr_pt;
    last_ctr_pt.position =
        control_points_.at(2).position +
        (control_points_.at(2).position - control_points_.at(1).position);
    control_points_.push_back(last_ctr_pt);
  }

  if (num_ctrl == 2) {
    LanePoint first_ctr_pt, last_ctr_pt;
    last_ctr_pt.position =
        control_points_.at(1).position +
        (control_points_.at(1).position - control_points_.at(0).position);
    first_ctr_pt.position =
        control_points_.at(0).position -
        (control_points_.at(1).position - control_points_.at(0).position);
    control_points_.insert(control_points_.begin(), first_ctr_pt);
    control_points_.push_back(last_ctr_pt);
  }
}

void LaneLandmark::referesh_lane_points_with_ctrl_points() {
  int num_points =
      static_cast<int>(ctrl_points_chord_ / downsample_distance_) + 1;
  Eigen::MatrixXd lane_points_mat = curve_line_->GetPoints(num_points);
  lane_points_.clear();
  for (int i = 0; i < lane_points_mat.rows(); ++i) {
    LanePoint pt;
    pt.position = lane_points_mat.row(i).head(3);
    lane_points_.push_back(pt);
  }
}

void LaneLandmark::find_border_point(
    const std::vector<LanePoint> &candidate_points,
    const LanePoint &query_point, std::set<int> &no_assigned,
    LanePoint &inner_border, LanePoint &outer_border,
    bool &outer_border_found) {
  double max_distance = .0;
  double min_distance = DBL_MAX;
  for (size_t i = 0; i < candidate_points.size(); ++i) {
    double distance =
        (candidate_points.at(i).position - query_point.position).norm();
    if (distance < ctrl_points_chord_) {
      auto it = no_assigned.find(i);
      no_assigned.erase(it);
      if (distance > max_distance) {
        max_distance = distance;
        inner_border = candidate_points.at(i);
      } else {
        if (distance < min_distance) {
          min_distance = distance;
          outer_border = candidate_points.at(i);
          outer_border_found = true;
        }
      }
    }
  }
}

std::vector<LanePoint> LaneLandmark::get_candidate_points(
    const std::vector<LanePoint> &lane_points) {
  if (control_points_.size() < 2) {
    return lane_points;
  } else {
    std::vector<LanePoint> candidate_points;
    size_t num_ctrl_point = control_points_.size();

    const auto &ctrl_pt_0 = get_control_point(0);  // first ctrl
    const auto &ctrl_pt_1 = get_control_point(1);  // second ctrl

    const auto &ctrl_pt_reverse_0 =
        get_control_point(num_ctrl_point - 1);  // last ctrl
    const auto &ctrl_pt_reverse_1 =
        get_control_point(num_ctrl_point - 2);  // second to last ctrl

    Eigen::Vector3d normal_a = ctrl_pt_0.position - ctrl_pt_1.position;
    normal_a.normalize();
    Eigen::Vector3d normal_b =
        ctrl_pt_reverse_0.position - ctrl_pt_reverse_1.position;
    normal_b.normalize();

    for (size_t i = 0; i < lane_points.size(); ++i) {
      Eigen::Vector3d d_a =
          (lane_points.at(i).position - ctrl_pt_0.position).normalized();
      Eigen::Vector3d d_b =
          (lane_points.at(i).position - ctrl_pt_reverse_0.position)
              .normalized();
      double cos_a = d_a.dot(normal_a);
      double cos_b = d_b.dot(normal_b);
      double cos_thresh = std::cos(Deg2Rad(candidate_angle_thresh_));
      if (cos_a > cos_thresh || cos_b > cos_thresh) {
        candidate_points.push_back(lane_points.at(i));
      }
    }

    return candidate_points;
  }
}

const LanePoint &LaneLandmark::get_control_point(const size_t id) {
  return control_points_.at(id);
}

void LaneLandmark::SetCategory(uint8_t category) { category_ = category; }

std::vector<LanePoint> LaneLandmark::GetLanePoints() { return lane_points_; }

uint8_t LaneLandmark::GetCategory() { return category_; }

void LaneLandmark::curve_fitting(const std::vector<LanePoint> &lane_points) {
  Eigen::MatrixXd data = ConstructDataMatrix(lane_points);
  Eigen::MatrixXd transformed_data;
  Eigen::Matrix3d data_rotation;
  Eigen::VectorXd target_axis = Eigen::Vector2d::UnitY();
  GetTransformedData(data, target_axis, transformed_data, data_rotation);

  Eigen::VectorXd poly_xy =
      CubicPolyFit(transformed_data.col(0), transformed_data.col(1));
  Eigen::VectorXd poly_xz =
      CubicPolyFit(transformed_data.col(0), transformed_data.col(2));

  poly_rotation_ = data_rotation;
  cubic_polynomials_xy_ = poly_xy;
  cubic_polynomials_xz_ = poly_xz;
}

}  // namespace mono_lane_mapping
