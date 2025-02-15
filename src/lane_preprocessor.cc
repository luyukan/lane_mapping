#include "lane_preprocessor.h"

namespace mono_lane_mapping {
LanePreprocessor &LanePreprocessor::GetInstance() {
  static LanePreprocessor instance;
  return instance;
}
LanePreprocessor::LanePreprocessor() {}

void LanePreprocessor::denoisePoints(
    const std::vector<LanePoint> &lane_points,
    std::vector<LanePoint> &denoised_lane_points) {
  Eigen::MatrixXd X = ConstructDataMatrix(lane_points);

  Eigen::VectorXd principleAxis;
  Eigen::MatrixXd transformed_X;
  Eigen::VectorXd targetAxis = Eigen::Vector2d::UnitY();
  Eigen::Matrix3d data_rotation;
  GetTransformedData(X, targetAxis, transformed_X, data_rotation);

  double min_x = transformed_X.col(0).minCoeff();
  double max_x = transformed_X.col(0).maxCoeff();

  Eigen::VectorXd coeff_xy =
      CubicPolyFit(transformed_X.col(0), transformed_X.col(1));
  Eigen::VectorXd coeff_xz =
      CubicPolyFit(transformed_X.col(0), transformed_X.col(2));

  double x = min_x;
  while (x < max_x) {
    double y = ApplyCubicPoly(x, coeff_xy);
    double z = ApplyCubicPoly(x, coeff_xz);
    LanePoint denoised_lane_point;
    denoised_lane_point.position =
        data_rotation.transpose() * Eigen::Vector3d(x, y, z);  // convert back
    denoised_lane_points.push_back(denoised_lane_point);
    x += downsample_distance_;
  }

  //  std::cout <<denoised_lane_points[0].position.transpose() << std::endl;
  //  std::cout <<lane_points[0].position.transpose() << std::endl;
  //  std::cout <<denoised_lane_points.back().position.transpose() << std::endl;
  //  std::cout <<lane_points.back().position.transpose() << std::endl;
  //  std::cout << "---------\n";
}

void LanePreprocessor::DenoiseLanePoints(
    const FrameObservation &frame_observation,
    FrameObservation &cur_frame_observation) {
  for (size_t i = 0; i < frame_observation.lane_observations.size(); ++i) {
    // if (frame_observation.lane_observations.at(i).lane_points.size() <
    //     observation_pts_num_min_) {
    //   continue;
    // }
    std::vector<LanePoint> denoised_lane_points;  // 拟合出来的车道线的点
    denoisePoints(frame_observation.lane_observations.at(i).lane_points,
                  denoised_lane_points);
    LaneObservation lane_observation;
    lane_observation.local_id =
        frame_observation.lane_observations.at(i).local_id;
    lane_observation.lane_points = denoised_lane_points;
    cur_frame_observation.lane_observations.push_back(lane_observation);
  }
}

void LanePreprocessor::Init() {
  SystemParam &system_param = SystemParam::GetInstance();
  downsample_distance_ =
      system_param.GetPreProcessParameters().downsample_distance;
  observation_pts_num_min_ =
      system_param.GetPreProcessParameters().observation_pts_num_min;
}

}  // namespace mono_lane_mapping
