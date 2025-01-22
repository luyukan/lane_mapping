#include "lane_landmark.h"

namespace mono_lane_mapping {
LaneLandmark::LaneLandmark() {}

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
  if(control_points_.empty()) {
    control_points_.push_back(lane_points.at(0));
//    ctrl_points_id.insert(0);
  }

//  while()

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
