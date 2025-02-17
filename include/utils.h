#pragma once
#include <math.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <iostream>

#include "type_define.h"

namespace mono_lane_mapping {

inline Eigen::VectorXd Pca(const Eigen::MatrixXd &data) {
  Eigen::VectorXd mean = data.colwise().mean();
  Eigen::MatrixXd centered = data.rowwise() - mean.transpose();
  Eigen::MatrixXd cov = (centered.transpose() * centered) / (data.rows() - 1);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);
  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  Eigen::MatrixXd eigenvectors = solver.eigenvectors();

  int max_index = 0;
  double max_value = eigenvalues[0];
  for (int i = 1; i < eigenvalues.size(); ++i) {
      if (eigenvalues[i] > max_value) {
          max_value = eigenvalues[i];
          max_index = i;
      }
  }
  return eigenvectors.col(max_index);
}

inline double Deg2Rad(const double deg) { return deg * (M_PI / 180.0); }

inline double Rad2Deg(const double rad) { return rad / M_PI * 180.0; }

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
  Eigen::Vector3d principleAx = Eigen::Vector3d::Zero();
  principleAx.head(2) = Pca(data.leftCols(2));
  Eigen::Vector3d targetAx = Eigen::Vector3d::Zero();
  targetAx.head(2) = target_axis;
  
  Eigen::Vector3d rotate_ax = principleAx.cross(targetAx);
  rotate_ax.normalize();
  double cos_theta = principleAx.dot(targetAx);
  double theta = std::acos(cos_theta);
  R = Eigen::AngleAxisd(theta, rotate_ax).toRotationMatrix();
  transformed_data = (R * data.transpose()).transpose();
}

}  // namespace mono_lane_mapping
