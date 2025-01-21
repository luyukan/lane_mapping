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

const LanePoint &LaneLandmark::get_latest_control_point_by_id(const int &id) {
  return control_points_.at(control_points_.size() - 1 - id);
}
const LanePoint &LaneLandmark::get_oldest_control_point_by_id(const int &id) {
  return control_points_.at(id);
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
    // std::cout << "Control Points Matrix:\n";
    // std::cout << points_mat << std::endl;
    if (curve_line_ != nullptr) {
      curve_line_.reset(new CatmullRomSplineList(points_mat, tau_));
    } else {
      curve_line_ = std::make_shared<CatmullRomSplineList>(points_mat, tau_);
    }
    refresh_lane_points_with_ctrl_points();
    refresh_kd_tree();
    refresh_ctrl_kd_tree();
  }
}

int LaneLandmark::GetId() { return id_; }

void LaneLandmark::SetId(int id) { id_ = id; }

void LaneLandmark::SetControlPosition(int id, const Eigen::Vector3d &position) {
  control_points_.at(id).position = position;
}

bool LaneLandmark::FindFootPoint(const LanePoint &measurement,
                                 const Odometry &pose, LaneLandmark::Ptr lm,
                                 std::vector<int> &ctrl_pts_id, double &u) {
  auto ctrl_tree = lm->GetCtrlSearchTree();
  Eigen::Vector3d twb = pose.twb;
  Eigen::Matrix3d Rwb = pose.qwb.toRotationMatrix();
  Eigen::Vector3d world_pt = Rwb * measurement.position + twb;
  const int k = 2;
  std::vector<int> indices;
  std::vector<double> distances;
  ctrl_tree->Query(world_pt, indices, distances, k);
  LanePoint pt_w;
  pt_w.position = world_pt;

  for (size_t i = 0; i < k; ++i) {
    if (distances.at(i) > ctrl_points_chord_) {
      return false;
    }
    if (indices.at(i) == 0 ||
        indices.at(i) == lm->GetControlPoints().size() - 1) {
      return false;
    }
  }

  if (abs(indices[0] - indices[1]) > 1) {
    return false;
  }

  int id0 = std::min(indices[0], indices[1]);
  int id1 = std::max(indices[0], indices[1]);

  ctrl_pts_id.push_back(id0 - 1);
  ctrl_pts_id.push_back(id0);
  ctrl_pts_id.push_back(id1);
  ctrl_pts_id.push_back(id1 + 1);

  std::vector<LanePoint> ctrl_pts4;
  ctrl_pts4.push_back(this->GetControlPoint(id0 - 1));
  ctrl_pts4.push_back(this->GetControlPoint(id0));
  ctrl_pts4.push_back(this->GetControlPoint(id1));
  ctrl_pts4.push_back(this->GetControlPoint(id1 + 1));

  double error;
  bool succ = parameterization(pt_w, ctrl_pts4, error, u);

  if(!succ) {
    return false;
  }

  if (error > 10.0) {
    return false;
  }

  return true;
}

bool LaneLandmark::UpdateCtrlPointsWithLaneObservation(
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

  CubicPolyLine cubic_line = curve_fitting(
      lane_points_world);  // 观测转到世界坐标系下拟合出来的三次曲线
  const LanePoint &initial_point = get_latest_control_point();
  std::cout << "UpdateCtrlPointsWithLaneObservation Now" << std::endl;
  get_skeleton_points(lane_points_world, cubic_line, initial_point, false);

  CatMullSmooth();

  // perform_quality_examine();
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

  CubicPolyLine cubic_line = curve_fitting(
      lane_points_world);  // 观测转到世界坐标系下拟合出来的三次曲线
  const LanePoint &initial_point = lane_points_world.at(0);
  std::cout << "InitCtrlPointsWithLaneObservation Now" << std::endl;
  get_skeleton_points(lane_points_world, cubic_line, initial_point, true);

  const auto &head_ctrl = get_oldest_control_point();
  const auto &tail_ctrl = get_latest_control_point();
  // convert to local coordinate system
  Eigen::Matrix3d Rbw = Rwb.transpose();
  Eigen::Vector3d tbw = -Rwb.transpose() * twb;

  Eigen::Vector3d head_ctrl_b = Rbw * head_ctrl.position + tbw;
  Eigen::Vector3d tail_ctrl_b = Rbw * tail_ctrl.position + tbw;

  if (head_ctrl_b.norm() > tail_ctrl_b.norm()) {
    revert_ctrl_points();
  }

  CatMullSmooth();

  // perform_quality_examine();
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

void LaneLandmark::append_ctrl_tail(const LanePoint &ctrl_pt) {
  control_points_.push_back(ctrl_pt);
}
void LaneLandmark::revert_ctrl_points() {
  std::reverse(control_points_.begin(), control_points_.end());
}

void LaneLandmark::append_ctrl_head(const LanePoint &ctrl_pt) {
  control_points_.push_front(ctrl_pt);
}

void LaneLandmark::get_skeleton_points(
    const std::vector<LanePoint> &lane_points, const CubicPolyLine &cubic_lane,
    const LanePoint &initial_point, bool initialization) {
  std::vector<LanePoint> internal_lane_points = lane_points;
  if (initialization) {
    control_points_.push_back(initial_point);
  }

  auto internal_initial_point = initial_point;
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
    LanePoint inner_border, outer_border;
    // inner_border是距离query点距离小于chord thresh的点中距离最远的点
    // outer_border是距离query点中距离大于chord thresh中距离最近的点
    // no_assigned 是origin_points中距离query点距离大于等于chord thresh的点的id
    bool outer_border_found{false};

    find_border_point(candidate_points, initial_point, no_assigned,
                      inner_border, outer_border, outer_border_found);

    const auto &oldest_ctrl = get_oldest_control_point();  // oldest ctrl
    const auto &latest_ctrl = get_latest_control_point();  // latest ctrl

    if (outer_border_found == false) {
      double distance_head =
          (oldest_ctrl.position - inner_border.position).norm();
      double distance_tail =
          (latest_ctrl.position - inner_border.position).norm();

      if (distance_head <=
          distance_tail) {  // first control point closer to the inner border
        if (control_points_.size() >= 2) {
          inner_border = get_query_point(get_second_oldest_control_point(),
                                         oldest_ctrl, cubic_lane);
        }
        LanePoint next_initial = get_next_node(inner_border, oldest_ctrl,
                                               ctrl_points_chord_, cubic_lane);
        append_ctrl_head(next_initial);
        // internal_initial_point = next_initial;

      } else {  // last control point closer to the inner border
        if (control_points_.size() >= 2) {
          inner_border = get_query_point(get_second_latest_control_point(),
                                         latest_ctrl, cubic_lane);
        }
        LanePoint next_initial = get_next_node(inner_border, latest_ctrl,
                                               ctrl_points_chord_, cubic_lane);
        append_ctrl_tail(next_initial);
        // internal_initial_point = next_initial;
      }
      return;
    } else {
      double distance_head =
          (oldest_ctrl.position - outer_border.position).norm();
      double distance_tail =
          (latest_ctrl.position - outer_border.position).norm();

      if (distance_head <= distance_tail) {
        if (control_points_.size() >= 2) {
          outer_border = get_query_point(get_second_oldest_control_point(),
                                         oldest_ctrl, cubic_lane);
        }
        LanePoint next_initial = get_next_node(outer_border, oldest_ctrl,
                                               ctrl_points_chord_, cubic_lane);
        append_ctrl_head(next_initial);
        // internal_initial_point = next_initial;

      } else {
        if (control_points_.size() >= 2) {
          outer_border = get_query_point(get_second_latest_control_point(),
                                         latest_ctrl, cubic_lane);
        }
        LanePoint next_initial = get_next_node(outer_border, latest_ctrl,
                                               ctrl_points_chord_, cubic_lane);
        append_ctrl_tail(next_initial);
        // internal_initial_point = next_initial;
      }

      internal_lane_points = update_lane_points(candidate_points, no_assigned);
    }
  }
}

LanePoint LaneLandmark::get_query_point(const LanePoint &start_point,
                                        const LanePoint &end_point,
                                        const CubicPolyLine &cublic_lane) {
  LanePoint point;
  Eigen::Vector3d start_point_transformed =
      cublic_lane.poly_rotation * start_point.position;
  Eigen::Vector3d end_point_transformed =
      cublic_lane.poly_rotation * end_point.position;
  double direction = end_point_transformed[0] - start_point_transformed[0];
  direction = direction > 0 ? 1.0 : -1.0;
  const double next_distance = 10.0;
  double x_query = end_point_transformed[0] + direction * next_distance;
  double y_query = ApplyCubicPoly(x_query, cublic_lane.cubic_polynomials_xy);
  double z_query = ApplyCubicPoly(x_query, cublic_lane.cubic_polynomials_xz);
  Eigen::Vector3d position = Eigen::Vector3d(x_query, y_query, z_query);
  // transform back
  point.position = cublic_lane.poly_rotation.transpose() * position;
  point.visibility = start_point.visibility;

  // std::cout << point.position.transpose() << "\n"
  //           << start_point.position.transpose() << "\n"
  //           << end_point.position.transpose() << "\n"
  //           << "@@@@@@@" << std::endl;

  return point;
}

const LanePoint &LaneLandmark::get_latest_control_point() {
  return control_points_.back();
}
const LanePoint &LaneLandmark::get_oldest_control_point() {
  return control_points_.front();
}
const LanePoint &LaneLandmark::get_second_latest_control_point() {
  return control_points_.at(control_points_.size() - 2);
}
const LanePoint &LaneLandmark::get_second_oldest_control_point() {
  return control_points_.at(1);
}
LanePoint LaneLandmark::get_next_node(const LanePoint &query_point,
                                      const LanePoint &center_point,
                                      double radius,
                                      const CubicPolyLine &cublic_lane) {
  LanePoint next_node;
  Eigen::Vector3d query_transformed =
      cublic_lane.poly_rotation * query_point.position;
  Eigen::Vector3d center_transformed =
      cublic_lane.poly_rotation * center_point.position;
  const int search_times = 10;
  Eigen::Vector3d last_result = Eigen::Vector3d::Zero();
  for (int i = 0; i < search_times; ++i) {
    Eigen::Vector3d nearst_on_circle = get_nearest_on_circle(
        query_transformed, center_transformed,
        radius);  // center_transformed
                  // 到query_transformed的射线和以center_transformed为球心，radius为半径的球面的交点
    double x = nearst_on_circle.x();
    double y = ApplyCubicPoly(x, cublic_lane.cubic_polynomials_xy);
    double z = ApplyCubicPoly(x, cublic_lane.cubic_polynomials_xz);
    query_transformed = Eigen::Vector3d(x, y, z);
    double delta = (query_transformed - last_result).norm();
    if (delta < 1e-2) {
      break;
    }
    last_result = query_transformed;
  }

  Eigen::Vector3d node_on_poly =
      cublic_lane.poly_rotation.transpose() * query_transformed;
  Eigen::Vector3d nearest_on_circle =
      get_nearest_on_circle(node_on_poly, center_point.position, radius);
  next_node.position = nearest_on_circle;
  return next_node;
}

bool LaneLandmark::parameterization(const LanePoint &point,
                                    const std::vector<LanePoint> &ctrl_points4,
                                    double &error, double &u) {
  Eigen::Matrix<double, 4, 3> mat;
  for (size_t i = 0; i < 4; ++i) {
    mat.row(i) = ctrl_points4.at(i).position.transpose();
  }

  CatmullRomSpline spline(mat);
  Eigen::MatrixXd anchors = spline.GetPoints(4);
  std::vector<double> ratio_vector{0.0, 1.0/3.0, 2.0/3.0, 1.0};

  KDTree tree;
  tree.ConstructTree(anchors.leftCols(3));

  std::vector<int> indices;
  std::vector<double> distances;
  tree.Query(point.position, indices, distances, 2);

  int id1 = std::min(indices[0], indices[1]);
  int id2 = std::max(indices[0], indices[1]);

  Eigen::Vector3d ctrl1 = ctrl_points4.at(id1).position;
  Eigen::Vector3d ctrl2 = ctrl_points4.at(id2).position;

  Eigen::Vector3d v1 = ctrl2 - ctrl1;
  Eigen::Vector3d v2 = point.position - ctrl1;

  double ratio = v1.dot(v2) / (v1.norm() * v1.norm());

  u = ratio_vector[id1] + ratio * (ratio_vector[3] - ratio_vector[id1]);
  if (u < 0 || u > 1) {
    return false;
  }

  Eigen::Vector3d est = spline.GetPoint(u).head(3);
  error = (est - point.position).norm();

  return true;
}

void LaneLandmark::perform_quality_examine() {
  std::cout << "+++++++++++++++++++++++++++++++++++++\n";
  if (control_points_.size() < 2) {
    return;
  }
  for (size_t i = 1; i < control_points_.size(); ++i) {
    std::cout << (control_points_.at(i).position -
                  control_points_.at(i - 1).position)
                     .normalized()
                     .transpose()
              << std::endl;
  }
  std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";
}

void LaneLandmark::refresh_ctrl_kd_tree() {
  if (ctrl_kd_tree_ == nullptr) {
    ctrl_kd_tree_ = std::make_shared<KDTree>();
  }
  ctrl_kd_tree_->Reset();
  Eigen::MatrixXd ctrl_pts_mat =
      Eigen::MatrixXd::Zero(control_points_.size(), 3);
  for (size_t i = 0; i < control_points_.size(); ++i) {
    ctrl_pts_mat.row(i) = control_points_.at(i).position.transpose();
  }
  ctrl_kd_tree_->ConstructTree(ctrl_pts_mat);
}

void LaneLandmark::refresh_kd_tree() {
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
        control_points_[2].position +
        (control_points_[2].position - control_points_[1].position);
    append_ctrl_tail(last_ctr_pt);
  }

  if (num_ctrl == 2) {
    LanePoint last_ctr_pt, first_ctr_pt;
    last_ctr_pt.position =
        control_points_[1].position +
        (control_points_[1].position - control_points_[0].position);
    first_ctr_pt.position =
        control_points_[0].position -
        (control_points_[1].position - control_points_[0].position);

    append_ctrl_head(first_ctr_pt);
    append_ctrl_tail(last_ctr_pt);
  }
}

void LaneLandmark::refresh_lane_points_with_ctrl_points() {
  int num_points =
      static_cast<int>(ctrl_points_chord_ / downsample_distance_) + 1;
  std::cout << "Control Points num: " << control_points_.size() << std::endl;
  Eigen::MatrixXd lane_points_mat = curve_line_->GetPoints(num_points);
  // std::cout << "lane points\n";
  // std::cout << lane_points_mat << std::endl;
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
  // std::cout << "Printing NoAssigned\n";
  // for (const auto &id : no_assigned) {
  //   std::cout << id << std::endl;
  // }
  // std::cout << "Printing NoAssigned End\n";

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
      }
    } else {
      if (distance < min_distance) {
        min_distance = distance;
        outer_border = candidate_points.at(i);
        outer_border_found = true;
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

    const auto &latest_ctrl = get_latest_control_point();  // latest ctrl
    const auto &second_latest_ctrl =
        get_second_latest_control_point();  // second latest ctrl

    const auto &oldest_ctrl = get_oldest_control_point();  // oldest ctrl
    const auto &second_oldest_ctrl =
        get_second_oldest_control_point();  // second oldest ctrl

    Eigen::Vector3d normal_a =
        oldest_ctrl.position - second_oldest_ctrl.position;
    normal_a.normalize();
    Eigen::Vector3d normal_b =
        latest_ctrl.position - second_latest_ctrl.position;
    normal_b.normalize();

    for (size_t i = 0; i < lane_points.size(); ++i) {
      Eigen::Vector3d d_a =
          (lane_points.at(i).position - oldest_ctrl.position).normalized();
      Eigen::Vector3d d_b =
          (lane_points.at(i).position - latest_ctrl.position).normalized();
      double cos_a = d_a.dot(normal_a);
      double cos_b = d_b.dot(normal_b);
      double cos_thresh = std::cos(Deg2Rad(candidate_angle_thresh_));
      if (cos_a > cos_thresh ||
          cos_b >
              cos_thresh) {  // 夹角足够小，只能处于第一个控制点之前，或者最后一个控制点之后【控制点是有序点的情况下】
        candidate_points.push_back(lane_points.at(i));
      }
    }

    return candidate_points;
  }
}

Eigen::Vector3d LaneLandmark::get_nearest_on_circle(
    const Eigen::Vector3d &query, const Eigen::Vector3d &center,
    const double radius) {
  Eigen::Vector3d dir = query - center;
  dir.normalize();
  Eigen::Vector3d intersection = center + radius * dir;
  return intersection;
}

const LanePoint &LaneLandmark::get_control_point(const size_t id) {
  return control_points_.at(id);
}

void LaneLandmark::SetCategory(uint8_t category) { category_ = category; }

std::vector<LanePoint> LaneLandmark::GetLanePoints() const {
  return lane_points_;
}

std::deque<LanePoint> LaneLandmark::GetControlPoints() const {
  return control_points_;
}

KDTree::Ptr LaneLandmark::GetCtrlSearchTree() { return ctrl_kd_tree_; }

LanePoint LaneLandmark::GetControlPoint(const int id) {
  return control_points_.at(id);
}

uint8_t LaneLandmark::GetCategory() { return category_; }

CubicPolyLine LaneLandmark::curve_fitting(
    const std::vector<LanePoint> &lane_points) {
  Eigen::MatrixXd data = ConstructDataMatrix(lane_points);
  Eigen::MatrixXd transformed_data;
  Eigen::Matrix3d data_rotation;
  Eigen::VectorXd target_axis = Eigen::Vector2d::UnitX();
  GetTransformedData(data, target_axis, transformed_data, data_rotation);
  // std::cout << transformed_data << std::endl;
  Eigen::VectorXd poly_xy =
      CubicPolyFit(transformed_data.col(0), transformed_data.col(1));
  Eigen::VectorXd poly_xz =
      CubicPolyFit(transformed_data.col(0), transformed_data.col(2));

  CubicPolyLine cubic_line;
  cubic_line.cubic_polynomials_xy = poly_xy;
  cubic_line.cubic_polynomials_xz = poly_xz;
  cubic_line.poly_rotation = data_rotation;

  return cubic_line;
}

}  // namespace mono_lane_mapping
