#include "catmull_rom_spline.h"

CatmullRomSpline::CatmullRomSpline(const Eigen::Matrix<double, 4, 3>& ctrl_pts,
                                   double tau)
    : ctrl_pts(ctrl_pts), tau(tau) {
  // Ensure there are exactly 4 control points at runtime
  assert(ctrl_pts.rows() == 4 &&
         "CatmullRomSpline requires exactly 4 control points");

  // Initialize the M matrix based on the tension parameter tau
  M << 0, 1, 0, 0, -tau, 0, tau, 0, 2 * tau, tau - 3, 3 - 2 * tau, -tau, -tau,
      2 - tau, tau - 2, tau;
}

Eigen::Matrix4d CatmullRomSpline::get_M() const { return M; }

Eigen::MatrixXd CatmullRomSpline::get_points(int num_points,
                                             bool return_knots) const {
  Eigen::MatrixXd points(num_points, ctrl_pts.cols());
  Eigen::VectorXd u_vec(4);  // [1, u, u^2, u^3] vector

  // Parameter u evenly spaced from 0 to 1
  Eigen::VectorXd u = Eigen::VectorXd::LinSpaced(num_points, 0, 1);

  for (int i = 0; i < num_points; ++i) {
    double t = u(i);
    u_vec << 1, t, t * t, t * t * t;
    points.row(i) = u_vec.transpose() * M * ctrl_pts;
  }

  if (return_knots) {
    // Append the parameter values (u) to the points
    Eigen::MatrixXd result(num_points, ctrl_pts.cols() + 1);
    result << points, u;
    return result;
  }

  return points;
}

Eigen::VectorXd CatmullRomSpline::get_point(double u, bool return_coeff) const {
  Eigen::VectorXd u_vec(4);
  u_vec << 1, u, u * u, u * u * u;

  if (!return_coeff) {
    return u_vec.transpose() * M * ctrl_pts;
  } else {
    Eigen::MatrixXd coeff = u_vec.transpose() * M;
    return (coeff * ctrl_pts).transpose();
  }
}

Eigen::VectorXd CatmullRomSpline::get_derivative(double u) const {
  Eigen::VectorXd u_vec(4);
  u_vec << 0, 1, 2 * u, 3 * u * u;

  Eigen::VectorXd derivative = u_vec.transpose() * M * ctrl_pts;
  return derivative.normalized();  // Normalize the derivative
}
