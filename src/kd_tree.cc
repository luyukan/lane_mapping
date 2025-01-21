#include "kd_tree.h"

namespace mono_lane_mapping {
KDTree::KDTree() {}

void KDTree::ConstructTree(const Eigen::MatrixXd &points) {
  // each row of points is a point
  kd_tree_ = std::make_shared<flann::Matrix<float>>(
      new float[points.rows() * points.cols()], points.rows(), points.cols());
  for (int i = 0; i < points.rows(); ++i) {
    for (int j = 0; j < points.cols(); ++j) {
      (*kd_tree_)[i][j] = static_cast<float>(points(i, j));
    }
  }
  kd_tree_index_ = std::make_shared<flann::Index<flann::L2<float>>>(
      *kd_tree_, flann::KDTreeIndexParams(4));

  kd_tree_index_->buildIndex();
}

void KDTree::Query(const Eigen::VectorXd &point, std::vector<int> &ids,
                   std::vector<double> &distance, int k) {
  flann::Matrix<int> indices(new int[k], 1, k);
  flann::Matrix<float> dists(new float[k], 1, k);

  flann::Matrix<float> flann_query(new float[point.size()], 1, point.size());

  for (int i = 0; i < point.size(); ++i) {
    flann_query[0][i] = static_cast<float>(point(i));
  }
  kd_tree_index_->knnSearch(flann_query, indices, dists, k,
                            flann::SearchParams());

  for (int i = 0; i < k; ++i) {
    ids.push_back(indices[0][i]);
    distance.push_back(dists[0][i]);
  }
}

void KDTree::QueryNearest(const Eigen::VectorXd &point, double &distance,
                          int &idx) {
  const int k = 1;
  flann::Matrix<int> indices(new int[k], 1, k);
  flann::Matrix<float> dists(new float[k], 1, k);

  flann::Matrix<float> flann_query(new float[point.size()], 1, point.size());

  for (int i = 0; i < point.size(); ++i) {
    flann_query[0][i] = static_cast<float>(point(i));
  }
  kd_tree_index_->knnSearch(flann_query, indices, dists, k,
                            flann::SearchParams());

  distance = dists[0][0];
  idx = indices[0][0];
}

void KDTree::QueryNearest(const std::vector<Eigen::VectorXd> &points,
                          std::vector<double> &distance,
                          std::vector<int> &idx) {
  const int k = 1;
  distance.resize(points.size());
  idx.resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    flann::Matrix<int> indices(new int[k], 1, k);
    flann::Matrix<float> dists(new float[k], 1, k);
    flann::Matrix<float> flann_query(new float[points[i].size()], 1,
                                     points[i].size());
    for (size_t j = 0; j < points.at(i).rows(); ++j) {
      flann_query[0][j] = static_cast<float>(points.at(i)[j]);
    }
    kd_tree_index_->knnSearch(flann_query, indices, dists, k,
                              flann::SearchParams());
    distance.at(i) = dists[0][0];
    idx.at(i) = indices[0][0];
  }
}

void KDTree::Reset() {
  if (kd_tree_ != nullptr) {
    kd_tree_.reset(new flann::Matrix<float>);
  }
}
}  // namespace mono_lane_mapping
