#include "catmull_rom_spline.h"

namespace mono_lane_mapping {
CatmullRomSpline::CatmullRomSpline(const Eigen::Matrix<double, 4, 3>& ctrl_pts,
                                   double tau)
    : ctrl_pts_(ctrl_pts), tau_(tau) {
  // Ensure there are exactly 4 control points at runtime
  assert(ctrl_pts.rows() == 4 &&
         "CatmullRomSpline requires exactly 4 control points");

  // Initialize the M matrix based on the tension parameter tau
  M_ << 0, 1, 0, 0, -tau_, 0, tau_, 0, 2 * tau_, tau_ - 3, 3 - 2 * tau_, -tau_,
      -tau_, 2 - tau_, tau_ - 2, tau_;
}

Eigen::Matrix4d CatmullRomSpline::GetM() const { return M_; }

Eigen::MatrixXd CatmullRomSpline::GetPoints(int num_points,
                                            bool return_knots) const {
  Eigen::MatrixXd points(num_points, ctrl_pts_.cols());
  Eigen::VectorXd u_vec(4);  // [1, u, u^2, u^3] vector

  // Parameter u evenly spaced from 0 to 1
  Eigen::VectorXd u = Eigen::VectorXd::LinSpaced(num_points, 0, 1);

  for (int i = 0; i < num_points; ++i) {
    double t = u(i);
    u_vec << 1, t, t * t, t * t * t;
    points.row(i) = u_vec.transpose() * M_ * ctrl_pts_;
  }

  if (return_knots) {
    // Append the parameter values (u) to the points
    Eigen::MatrixXd result(num_points, ctrl_pts_.cols() + 1);
    result << points, u;
    return result;
  }

  return points;
}

Eigen::VectorXd CatmullRomSpline::GetPoint(double u) const {
  Eigen::VectorXd u_vec(4);
  u_vec << 1, u, u * u, u * u * u;

  Eigen::MatrixXd coeff = u_vec.transpose() * M_;
  return (coeff * ctrl_pts_).transpose();
}


Eigen::VectorXd CatmullRomSpline::GetCoeff(double u) const {
  Eigen::VectorXd u_vec(4);
  u_vec << 1, u, u * u, u * u * u;

  Eigen::VectorXd coeff = (u_vec.transpose() * M_).transpose();
  return coeff;
}

Eigen::VectorXd CatmullRomSpline::GetDerivative(double u) const {
  Eigen::VectorXd u_vec(4);
  u_vec << 0, 1, 2 * u, 3 * u * u;

  Eigen::VectorXd derivative = u_vec.transpose() * M_ * ctrl_pts_;
  return derivative.normalized();  // Normalize the derivative
}
}  // namespace mono_lane_mapping
