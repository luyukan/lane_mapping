#pragma once
#include <cassert>
#include <memory>
#include <Eigen/Eigen>
namespace mono_lane_mapping {
class CatmullRomSpline {
 public:
  typedef std::shared_ptr<CatmullRomSpline> Ptr;
  // Constructor: Takes control points and an optional tension parameter
  // (default 0.5)
  CatmullRomSpline(const Eigen::Matrix<double, 4, 3>& ctrl_pts,
                   double tau = 0.5);

  // Get the M matrix used for Catmull-Rom interpolation
  Eigen::Matrix4d GetM() const;

  // Get interpolated points for a set of parameter values
  Eigen::MatrixXd GetPoints(int num_points, bool return_knots = false) const;

  // Get a single interpolated point at a given parameter u
  Eigen::VectorXd GetPoint(double u) const;

  Eigen::VectorXd GetCoeff(double u) const;

  // Get the derivative of the spline at a given parameter u
  Eigen::VectorXd GetDerivative(double u) const;

 private:
  Eigen::Matrix4d M_;  // The 4x4 M matrix used in Catmull-Rom interpolation
  Eigen::Matrix<double, 4, 3>
      ctrl_pts_;     // The fixed-size control points (4x3 matrix)
  double tau_{0.5};  // Tension parameter
};
}  // namespace mono_lane_mapping
