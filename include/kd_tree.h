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
  void Query(const Eigen::VectorXd &point, std::vector<int> &ids,
             std::vector<double> &distance, int k);
  void QueryNearest(const Eigen::VectorXd &point, double &distance, int &idx);
  void QueryNearest(const std::vector<Eigen::VectorXd> &points,
             std::vector<double> &distance, std::vector<int> &idx);
  void Reset();

 private:
  std::shared_ptr<flann::Matrix<float>> kd_tree_;
  std::shared_ptr<flann::Index<flann::L2<float>>> kd_tree_index_;
};
}  // namespace mono_lane_mapping