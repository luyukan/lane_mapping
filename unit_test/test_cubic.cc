#include <iostream>
#include <Eigen/Eigen>
#include "utils.h"

using namespace mono_lane_mapping;

int main() { 
    Eigen::MatrixXd data = Eigen::MatrixXd::Zero(20, 3);
    for (int i = 0; i < data.rows(); ++i) {
        double x = 0.7 * i;
        double y = 1.0 * x + 3;
        Eigen::Vector3d pt(x, y, 0);
        // pt.head(2) += Eigen::Vector2d::Random() * 0.2;
        data.row(i) = pt.transpose();
    } 

    std::cout << data << std::endl;
    Eigen::VectorXd target_ax = Eigen::Vector2d::UnitX();
    Eigen::MatrixXd transformed_data;
    Eigen::Matrix3d rotation;
    GetTransformedData(data, target_ax, transformed_data, rotation);
    std::cout << "----------\n";
    std::cout << transformed_data << std::endl;
    return 0;
 }