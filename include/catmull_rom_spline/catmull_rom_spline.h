#ifndef CATMULL_ROM_SPLINE_H
#define CATMULL_ROM_SPLINE_H

#include <Eigen/Dense>
#include <cassert>

class CatmullRomSpline {
 public:
  // Constructor: Takes control points and an optional tension parameter
  // (default 0.5)
  CatmullRomSpline(const Eigen::Matrix<double, 4, 3>& ctrl_pts,
                   double tau = 0.5);

  // Get the M matrix used for Catmull-Rom interpolation
  Eigen::Matrix4d get_M() const;

  // Get interpolated points for a set of parameter values
  Eigen::MatrixXd get_points(int num_points, bool return_knots = false) const;

  // Get a single interpolated point at a given parameter u
  Eigen::VectorXd get_point(double u, bool return_coeff = false) const;

  // Get the derivative of the spline at a given parameter u
  Eigen::VectorXd get_derivative(double u) const;

 private:
  Eigen::Matrix4d M;  // The 4x4 M matrix used in Catmull-Rom interpolation
  Eigen::Matrix<double, 4, 3>
      ctrl_pts;     // The fixed-size control points (4x3 matrix)
  double tau{0.5};  // Tension parameter
};

#endif  // CATMULL_ROM_SPLINE_H
