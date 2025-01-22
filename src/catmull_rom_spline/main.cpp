#include <iostream>
#include <Eigen/Dense>
#include "catmull_rom_spline.h"

int main() {
    // Define 4 control points (for example, in 3D space)
    Eigen::Matrix<double, 4, 3> ctrl_pts;
    ctrl_pts << 0, 0, 0,
                1, 0, 0,
                3, 0, 0,
                4, 0, 0;

    // Create CatmullRomSpline object with control points and tension parameter
    CatmullRomSpline spline(ctrl_pts, 0.5);

    // Get and print M matrix
    std::cout << "M matrix:\n" << spline.get_M() << std::endl;

    // Get and print 10 interpolated points
    Eigen::MatrixXd points = spline.get_points(10);
    std::cout << "\nInterpolated points (10 points):\n" << points << std::endl;

    // Get and print a single point at u = 0.5
    Eigen::VectorXd point = spline.get_point(0.5);
    std::cout << "\nInterpolated point at u = 0.5:\n" << point.transpose() << std::endl;

    // Get and print the derivative at u = 0.5
    Eigen::VectorXd derivative = spline.get_derivative(0.5);
    std::cout << "\nDerivative at u = 0.5:\n" << derivative.transpose() << std::endl;

    return 0;
}
