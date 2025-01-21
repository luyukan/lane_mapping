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
  Eigen::MatrixXd X = constructDataMatrix(lane_points);

  Eigen::VectorXd principleAxis = pca(X.leftCols(2));
  Eigen::VectorXd targetAxis = Eigen::Vector2d::UnitY();
  double dot = principleAxis.dot(targetAxis);
  double cross = principleAxis.x() * targetAxis.y() - principleAxis.y() * targetAxis.x();
  double angle = atan2(cross, dot);
  Eigen::Matrix3d R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::MatrixXd transformed_X = (R * X.transpose()).transpose();

  double min_x = transformed_X.col(0).minCoeff();
  double max_x = transformed_X.col(0).maxCoeff();

  Eigen::VectorXd coeff_xy = CubicPolyFit(transformed_X.col(0), transformed_X.col(1));
  Eigen::VectorXd coeff_xz = CubicPolyFit(transformed_X.col(0), transformed_X.col(2));

  double x = min_x;
  while(x < max_x) {
    double y = ApplyCubicPoly(x, coeff_xy);
    double z = ApplyCubicPoly(x, coeff_xz);
    LanePoint denoised_lane_point;
    denoised_lane_point.point_wcs = R.transpose() * Eigen::Vector3d(x, y, z); // convert back
    denoised_lane_points.push_back(denoised_lane_point);
    x += downsample_distance_;
  }

//  std::cout <<denoised_lane_points[0].point_wcs.transpose() << std::endl;
//  std::cout <<lane_points[0].point_wcs.transpose() << std::endl;
//  std::cout <<denoised_lane_points.back().point_wcs.transpose() << std::endl;
//  std::cout <<lane_points.back().point_wcs.transpose() << std::endl;
//  std::cout << "---------\n";

}


Eigen::VectorXd LanePreprocessor::pca(const Eigen::MatrixXd& data) {
  Eigen::VectorXd mean = data.colwise().mean();
  Eigen::MatrixXd centered = data.rowwise() - mean.transpose();
  Eigen::MatrixXd cov = (centered.transpose() * centered) / (data.rows() - 1);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);
  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  Eigen::MatrixXd eigenvectors = solver.eigenvectors();

  return eigenvectors.col(0);

}

void LanePreprocessor::DenoiseLanePoints(
    const FrameObservation &frame_observation,
    FrameObservation &cur_frame_observation) {
  for (size_t i = 0; i < frame_observation.lane_observations.size(); ++i) {
    std::vector<LanePoint> denoised_lane_points;
    denoisePoints(frame_observation.lane_observations.at(i).lane_points, denoised_lane_points);
    LaneObservation lane_observation;
    lane_observation.local_id = frame_observation.lane_observations.at(i).local_id;
    lane_observation.lane_points = denoised_lane_points;
    cur_frame_observation.lane_observations.push_back(lane_observation);
  }
}

Eigen::MatrixXd LanePreprocessor::constructDataMatrix(
    const std::vector<LanePoint> &lane_points) {
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(lane_points.size(), 3);
  for (size_t i = 0; i < lane_points.size(); ++i) {
    Eigen::Vector3d point = lane_points.at(i).point_wcs;
    X.row(i) = point.transpose();
  }

  return X;
}

void LanePreprocessor::Init(const std::string &config) {
  try {
    YAML::Node yml = YAML::LoadFile(config);
    YAML::Node preprocess_node = yml["preprocess"];
    downsample_distance_ = preprocess_node["downsample_distance"].as<double>();
  } catch (const YAML::Exception &e) {
    std::cerr << "Error reading YAML: " << e.what() << std::endl;
  }
}

}  // namespace mono_lane_mapping
