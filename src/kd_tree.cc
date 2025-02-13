#include "kd_tree.h"

namespace mono_lane_mapping {
KDTree::KDTree() {}

void KDTree::ConstructTree(const Eigen::MatrixXd &points) {
  // each row of points is a point
  kd_tree_ = std::make_shared<flann::Matrix<double>>(
      const_cast<double *>(points.data()), points.rows(), points.cols());

  kd_tree_index_ = std::make_shared<flann::Index<flann::L2<double>>>(
      *kd_tree_, flann::KDTreeIndexParams(1));

  kd_tree_index_->buildIndex();
}

void KDTree::Query(const Eigen::VectorXd &point, double &distance, int &idx,
                   int k) {
  std::vector<std::vector<int>> indices;  // Nearest neighbor indices
  std::vector<std::vector<double>> dists;

  flann::Matrix<double> flann_query(const_cast<double *>(point.data()), 1,
                                    point.rows());

  kd_tree_index_->knnSearch(flann_query, indices, dists, k,
                            flann::SearchParams());

  distance = dists[0][0];
  idx = indices[0][0];
}

void KDTree::Query(const std::vector<Eigen::VectorXd> &points,
                   std::vector<double> &distance, std::vector<int> &idx,
                   int k) {
  flann::Matrix<double> flann_query(
      new double[points.size() * points[0].size()], points.size(),
      points[0].size());

  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = 0; j < points.at(0).rows(); ++j) {
      flann_query[i][j] = points.at(i)[j];
    }
  }

  distance.resize(points.size());
  idx.resize(points.size());

  flann::Matrix<int> indices(new int[points.size() * k], points.size(), k);
  flann::Matrix<double> dists(new double[points.size() * k], points.size(), k);

  kd_tree_index_->knnSearch(flann_query, indices, dists, k,
                            flann::SearchParams());

  for (size_t i = 0; i < points.size(); ++i) {
    distance.at(i) = dists[i][0];
    idx.at(i) = indices[i][0];
  }
}

void KDTree::Reset() {
  if (kd_tree_ != nullptr) {
    kd_tree_.reset(new flann::Matrix<double>);
  }
}
}  // namespace mono_lane_mapping
