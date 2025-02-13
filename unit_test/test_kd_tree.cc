#include "kd_tree.h"

using namespace mono_lane_mapping;
int main() {
  KDTree tree;
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(5, 3);
  for (int i = 0; i < points.rows(); ++i) {
    points(i, 0) = i;
  }

  std::cout << points << std::endl;
  tree.ConstructTree(points);

  // query single points
  Eigen::VectorXd query_point = Eigen::VectorXd::Zero(3);
  query_point[0] = 9.3;

  double distance;
  int idx;
  tree.Query(query_point, distance, idx);
  std::cout << "-----------\n";
  std::cout << distance << " " << idx << std::endl;

  // query multiple point

  std::vector<Eigen::VectorXd> query_points;
  for (int i = 0; i < 10; ++i) {
    Eigen::VectorXd q_point = Eigen::VectorXd::Zero(3);
    q_point[0] = 0.3 + i;
    query_points.push_back(q_point);
}

  std::vector<double> distances;
  std::vector<int> ids;
  tree.Query(query_points, distances, ids);
  std::cout << "-----------\n";
  for (size_t i = 0; i < distances.size(); ++i) {
    std::cout << distances.at(i) << " == " << ids.at(i) << std::endl;
  }
  return 0;
}