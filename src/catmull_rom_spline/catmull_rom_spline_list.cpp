#include "catmull_rom_spline_list.h"

CatmullRomSplineList::CatmullRomSplineList(Eigen::MatrixXd input_ctrl_pts,
                                           double tau)
    : tau(tau) {
  // Initialize the private member ctrl_pts
  ctrl_pts = input_ctrl_pts;

  if (ctrl_pts.rows() < 4) {
    ctrl_pts =
        padding(ctrl_pts);  // Pad control points if there are less than 4
  }
  num_ctrl_pts = ctrl_pts.rows();

  // Initialize the M matrix based on the tension parameter tau
  M << 0, 1, 0, 0, -tau, 0, tau, 0, 2 * tau, tau - 3, 3 - 2 * tau, -tau, -tau,
      2 - tau, tau - 2, tau;
}

Eigen::Matrix4d CatmullRomSplineList::get_M() const { return M; }

Eigen::MatrixXd CatmullRomSplineList::get_points(int num_points) const {
  std::vector<Eigen::MatrixXd> points_list;

  for (int i = 0; i < num_ctrl_pts - 3; ++i) {
    Eigen::MatrixXd four_ctrl_pts = ctrl_pts.middleRows(i, 4);
    CatmullRomSpline spline(four_ctrl_pts, tau);
    if (i != num_ctrl_pts - 4) {
      points_list.push_back(spline.get_points(num_points)
                                .topRows(num_points - 1));  // Avoid overlap
    } else {
      points_list.push_back(spline.get_points(num_points));
    }
  }

  // Concatenate all point sets into one matrix
  Eigen::MatrixXd all_points(points_list.size() * num_points, ctrl_pts.cols());
  int row_idx = 0;
  for (const auto& points : points_list) {
    all_points.middleRows(row_idx, points.rows()) = points;
    row_idx += points.rows();
  }

  return all_points;
}

Eigen::MatrixXd CatmullRomSplineList::padding(
    const Eigen::MatrixXd& ctrl_pts) const {
  Eigen::MatrixXd padded_ctrl_pts = ctrl_pts;

  if (ctrl_pts.rows() == 3) {
    // Extrapolate last point
    Eigen::RowVectorXd last_pt =
        ctrl_pts.row(2) + (ctrl_pts.row(2) - ctrl_pts.row(1));
    padded_ctrl_pts.conservativeResize(ctrl_pts.rows() + 1, ctrl_pts.cols());
    padded_ctrl_pts.row(3) = last_pt;
  } else if (ctrl_pts.rows() == 2) {
    // Extrapolate first and last points
    Eigen::RowVectorXd last_pt =
        ctrl_pts.row(1) + (ctrl_pts.row(1) - ctrl_pts.row(0));
    Eigen::RowVectorXd first_pt =
        ctrl_pts.row(0) - (ctrl_pts.row(1) - ctrl_pts.row(0));
    padded_ctrl_pts.conservativeResize(ctrl_pts.rows() + 2, ctrl_pts.cols());
    padded_ctrl_pts.row(0) = first_pt;
    padded_ctrl_pts.row(1) = ctrl_pts.row(0);
    padded_ctrl_pts.row(2) = ctrl_pts.row(1);
    padded_ctrl_pts.row(3) = last_pt;
  } else {
    throw std::invalid_argument("ctrl_pts should have at least 2 points");
  }

  return padded_ctrl_pts;
}