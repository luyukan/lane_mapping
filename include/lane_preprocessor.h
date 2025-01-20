#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "type_define.h"

namespace mono_lane_mapping {
class LanePreprocessor {
 public:
  typedef std::shared_ptr<LanePreprocessor> Ptr;
  static LanePreprocessor &GetInstance();
  LanePreprocessor();
  void Init(const std::string &config);
  void DenoiseLanePoints(const FrameObservation &frame_observation,
                         FrameObservation &cur_frame_observation);

 private:
  void denoisePoints(const std::vector<LanePoint> &lane_points, std::vector<LanePoint> &denoised_lane_points);
  Eigen::MatrixXd constructDataMatrix(const std::vector<LanePoint> &lane_points);
  Eigen::VectorXd pca(const Eigen::MatrixXd& data);
  Eigen::VectorXd cubicPolyFix(const Eigen::VectorXd &x, const Eigen::VectorXd &y);
  double applyCubicPoly(const double x, const Eigen::VectorXd &coeff);
  double downsample_distance_{0.0};
};
}  // namespace mono_lane_mapping
