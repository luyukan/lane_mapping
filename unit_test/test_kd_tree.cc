#include "kd_tree.h"

using namespace mono_lane_mapping;
int main() {
  KDTree tree;
  // //   Eigen::MatrixXd points = Eigen::MatrixXd::Zero(5, 3);
  // //   for (int i = 0; i < points.rows(); ++i) {
  // //     points(i, 0) = i;
  // //   }

  // //   std::cout << points << std::endl;
  // //   tree.ConstructTree(points);

  // //   // query single points
  // //   Eigen::VectorXd query_point = Eigen::VectorXd::Zero(3);
  // //   query_point[0] = 9.3;

  // //   double distance;
  // //   int idx;
  // //   tree.Query(query_point, distance, idx);
  // //   std::cout << "-----------\n";
  // //   std::cout << distance << " " << idx << std::endl;

  // //   // query multiple point

  // //   std::vector<Eigen::VectorXd> query_points;
  // //   for (int i = 0; i < 10; ++i) {
  // //     Eigen::VectorXd q_point = Eigen::VectorXd::Zero(3);
  // //     q_point[0] = 0.3 + i;
  // //     query_points.push_back(q_point);
  // // }

  // //   std::vector<double> distances;
  // //   std::vector<int> ids;
  // //   tree.Query(query_points, distances, ids);
  // //   std::cout << "-----------\n";
  // //   for (size_t i = 0; i < distances.size(); ++i) {
  // //     std::cout << distances.at(i) << " == " << ids.at(i) << std::endl;
  // //   }

  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(19, 3);
  points << 8429.14, 226.589, -203.094, 8429.12, 227.089, -203.091, 8429.11,
      227.589, -203.087, 8429.1, 228.089, -203.084, 8429.09, 228.589, -203.081,
      8429.07, 229.088, -203.078, 8429.06, 229.588, -203.074, 8429.05, 230.088,
      -203.071, 8429.04, 230.588, -203.068, 8429.03, 231.088, -203.065, 8429.02,
      231.588, -203.063, 8429.00, 232.088, -203.06, 8428.99, 232.588, -203.057,
      8428.98, 233.087, -203.055, 8428.97, 233.587, -203.053, 8428.96, 234.087,
      -203.05, 8428.96, 234.587, -203.048, 8428.95, 235.087, -203.047, 8428.94,
      235.587, -203.045;

  tree.ConstructTree(points);

  std::cout << "==========\n";
  int id;
  double distance;
  Eigen::VectorXd point = Eigen::VectorXd::Zero(3);
  point << 8429.00, 232.088, -207.094;
  tree.QueryNearest(point, distance, id);
  std::cout << distance << " " << id << std::endl;

  std::vector<int> ids;
  std::vector<double> distances;
  std::vector<Eigen::VectorXd> pts;
  pts.push_back(point);
  point << 8429.1, 228.089, -203.084;
  pts.push_back(point);
  tree.QueryNearest(pts, distances, ids);
  std::cout << "=========================\n";
  for (size_t i = 0; i < pts.size(); ++i) {
    std::cout << distances.at(i) << " " << ids.at(i) << std::endl;
  }

  
  return 0;
}