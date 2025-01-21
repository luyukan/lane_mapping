#include <iostream>
#include <Eigen/Eigen>
#include "catmull_rom_spline_list.h"

using namespace mono_lane_mapping;

int main() {
    // Define control points (4 points for a single spline)
    Eigen::MatrixXd ctrl_pts(4, 3);
    ctrl_pts << 0.0, 0.0, 0.0,
                1.0, 0.0, 0.0,
                2.0, 0.0, 0.0,
                3.0, 0.0, 0.0;

    // Instantiate CatmullRomSplineList with control points and tension parameter
    double tau = 0.5;
    CatmullRomSplineList spline_list(ctrl_pts, tau);

    // Get the M matrix
    std::cout << "M Matrix for Spline List:\n" << spline_list.GetM() << std::endl;

    // Generate interpolated points for the spline list
    int num_points = 100;
    Eigen::MatrixXd points = spline_list.GetPoints(num_points);

    std::cout << "Generated " << points.rows() << " interpolated points:\n";
    std::cout << points << std::endl;

    // Test for control points with fewer than 4
    Eigen::MatrixXd fewer_pts(3, 3);
    fewer_pts << 0.0, 0.0, 0.0,
                 1.0, 0.0, 0.0,
                 2.0, 0.0, 0.0;

    // Instantiate a new CatmullRomSplineList with fewer control points
    CatmullRomSplineList spline_list_fewer(fewer_pts, tau);
    Eigen::MatrixXd points_fewer = spline_list_fewer.GetPoints(num_points);
    std::cout << "Generated points for fewer control points:\n";
    std::cout << points_fewer << std::endl;

    return 0;
}