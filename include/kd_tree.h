#pragma once

#include <Eigen/Eigen>
#include <flann/flann.hpp>
#include <iostream>
#include <memory>

namespace mono_lane_mapping {
class KDTree {
 public:
  typedef std::shared_ptr<KDTree> Ptr;
  KDTree();
  void ConstructTree(const Eigen::MatrixXd &points);
  void Query(const Eigen::VectorXd &point, double &distance, int &idx,
             int k = 1);
  void Query(const std::vector<Eigen::VectorXd> &points,
             std::vector<double> &distance, std::vector<int> &idx, int k = 1);
  void Reset();

 private:
  std::shared_ptr<flann::Matrix<double>> kd_tree_;
  std::shared_ptr<flann::Index<flann::L2<double>>> kd_tree_index_;
};
}  // namespace mono_lane_mapping