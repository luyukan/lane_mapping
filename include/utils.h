#pragma once

#include <Eigen/Eigen>

#include "type_define.h"

namespace mono_lane_mapping {


inline Eigen::VectorXd Pca(const Eigen::MatrixXd& data) {
  Eigen::VectorXd mean = data.colwise().mean();
  Eigen::MatrixXd centered = data.rowwise() - mean.transpose();
  Eigen::MatrixXd cov = (centered.transpose() * centered) / (data.rows() - 1);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);
  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  Eigen::MatrixXd eigenvectors = solver.eigenvectors();

  return eigenvectors.col(0);
}



inline Eigen::VectorXd CubicPolyFit(const Eigen::VectorXd &x,
                                    const Eigen::VectorXd &y) {
  int n_x = x.rows();
  int n_y = y.rows();

  if (n_x != n_y) {
    std::cout << "Cubic PolyFit Error: x and y size not equal\n";
  }

  Eigen::MatrixXd A(n_x, 4);
  for (int i = 0; i < n_x; ++i) {
    A(i, 0) = 1.0;
    A(i, 1) = x[i];
    A(i, 2) = x[i] * x[i];
    A(i, 3) = x[i] * x[i] * x[i];
  }

  Eigen::VectorXd coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * y);

  return coeffs;
}

inline Eigen::MatrixXd ConstructDataMatrix(
    const std::vector<LanePoint> &lane_points) {
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(lane_points.size(), 3);
  for (size_t i = 0; i < lane_points.size(); ++i) {
    Eigen::Vector3d point = lane_points.at(i).position;
    X.row(i) = point.transpose();
  }

  return X;
}

inline double ApplyCubicPoly(const double x, const Eigen::VectorXd &coeff) {
  if (coeff.rows() != 4) {
    std::cout << "Cubic Coeff Wrong\n";
  }

  Eigen::VectorXd multiplier = Eigen::VectorXd::Ones(4);
  multiplier[0] = 1.0;
  multiplier[1] = x;
  multiplier[2] = x * x;
  multiplier[3] = x * x * x;

  return multiplier.dot(coeff);
}

inline void GetTransformedData(const Eigen::MatrixXd &data,
                               const Eigen::VectorXd &target_axis,
                               Eigen::MatrixXd &transformed_data,
                               Eigen::Matrix3d &R) {

  Eigen::VectorXd principleAxis = Pca(data.leftCols(2));
  double dot = principleAxis.dot(target_axis);
  double cross = principleAxis.x() * target_axis.y() - principleAxis.y() * target_axis.x();
  double angle = atan2(cross, dot);
  R = Eigen::AngleAxisd(-angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transformed_data = (R * data.transpose()).transpose();

}

}  // namespace mono_lane_mapping
