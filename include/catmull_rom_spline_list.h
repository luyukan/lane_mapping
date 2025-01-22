#pragma once
#include <cassert>
#include <vector>

#include <Eigen/Eigen>

#include "catmull_rom_spline.h"  // Include the CatmullRomSpline class

namespace mono_lane_mapping {
class CatmullRomSplineList {
 public:
  // Constructor: Takes control points and an optional tension parameter
  CatmullRomSplineList(Eigen::MatrixXd ctrl_pts,
                       double tau = 0.5);  // No const here

  // Get the M matrix used for Catmull-Rom interpolation
  Eigen::Matrix4d GetM() const;

  // Get interpolated points for the entire list of control points
  Eigen::MatrixXd GetPoints(int num_points) const;

 private:
  // Helper function for padding control points
  Eigen::MatrixXd padding(const Eigen::MatrixXd& ctrl_pts) const;

  Eigen::Matrix4d M_;  // The 4x4 M matrix used in Catmull-Rom interpolation
  Eigen::MatrixXd ctrl_pts_;  // No const qualifier here
  int num_ctrl_pts_;          // Number of control points
  double tau_;                // Tension parameter
};
}  // namespace mono_lane_mapping


