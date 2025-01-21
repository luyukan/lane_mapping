#include <catmull_rom_spline.h>

#include <iostream>

using namespace mono_lane_mapping;

Eigen::Vector3d GetSplinePoint(const std::vector<Eigen::Vector3d> &ctrl_points,
                               const double u) {
  Eigen::Matrix<double, 4, 3> mat;
  for (size_t i = 0; i < 4; ++i) {
    mat.row(i) = ctrl_points.at(i).transpose();
  }

  CatmullRomSpline spline(mat);
  //   std::cout << mat << std::endl;
  //   std::cout << "------------------" << std::endl;

  return spline.GetPoint(u).head(3);
}
int main() {
  std::vector<Eigen::Vector3d> ctrl_points;
  ctrl_points.push_back(Eigen::Vector3d(1, 3, 2));
  ctrl_points.push_back(Eigen::Vector3d(2, 5, 4));
  ctrl_points.push_back(Eigen::Vector3d(3, 6, 3));
  ctrl_points.push_back(Eigen::Vector3d(6, 9, 5));

  Eigen::Matrix<double, 4, 3> mat;
  for (size_t i = 0; i < 4; ++i) {
    mat.row(i) = ctrl_points.at(i).transpose();
  }

  CatmullRomSpline spline(mat);


  const double u = 0.6;
  auto coeff = spline.GetCoeff(u);
  std::cout << "coeff: " << coeff.transpose() << std::endl;

  int jacobian_id = 0;
  double sigma = 1e-7;
  Eigen::Matrix3d delta = Eigen::Matrix3d::Identity() * sigma;

  Eigen::Matrix3d J = Eigen::Matrix3d::Zero();

  for (int i = 0; i < 3; ++i) {
    std::vector<Eigen::Vector3d> ctrl_points_plus = ctrl_points;
    std::vector<Eigen::Vector3d> ctrl_points_minus = ctrl_points;

    ctrl_points_plus.at(jacobian_id) += delta.col(i);
    ctrl_points_minus.at(jacobian_id) -= delta.col(i);

    Eigen::Vector3d point_plus = GetSplinePoint(ctrl_points_plus, u);
    Eigen::Vector3d point_minus = GetSplinePoint(ctrl_points_minus, u);

    double distortion = 2.0 * sigma;
    J.col(i) = (point_plus - point_minus) / distortion;
  }

  std::cout << J << std::endl;

  return 0;
}