#ifndef CATMULL_ROM_SPLINE_LIST_H
#define CATMULL_ROM_SPLINE_LIST_H

#include <Eigen/Dense>
#include <cassert>
#include <vector>

#include "catmull_rom_spline.h"  // Include the CatmullRomSpline class

class CatmullRomSplineList {
 public:
  // Constructor: Takes control points and an optional tension parameter
  CatmullRomSplineList(Eigen::MatrixXd ctrl_pts,
                       double tau = 0.5);  // No const here

  // Get the M matrix used for Catmull-Rom interpolation
  Eigen::Matrix4d get_M() const;

  // Get interpolated points for the entire list of control points
  Eigen::MatrixXd get_points(int num_points) const;

 private:
  // Helper function for padding control points
  Eigen::MatrixXd padding(const Eigen::MatrixXd& ctrl_pts) const;

  Eigen::Matrix4d M;  // The 4x4 M matrix used in Catmull-Rom interpolation
  Eigen::MatrixXd ctrl_pts;  // No const qualifier here
  int num_ctrl_pts;          // Number of control points
  double tau;                // Tension parameter
};

#endif  // CATMULL_ROM_SPLINE_LIST_H
