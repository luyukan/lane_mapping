#include <Eigen/Eigen>

namespace mono_lane_mapping {
inline Eigen::VectorXd CubicPolyFit(const Eigen::VectorXd &x,
                                    const Eigen::VectorXd &y) {
  int n_x = x.rows();
  int n_y = y.rows();

  if (n_x != n_y) {
    std::cout << "Cubic PolyFit Error: x and y size not equal\n";
  }

  Eigen::MatrixXd A(n_x, 4);
  for (int i = 0; i < n_x; ++i) {
    A(i, 0) = 1.0;
    A(i, 1) = x[i];
    A(i, 2) = x[i] * x[i];
    A(i, 3) = x[i] * x[i] * x[i];
  }

  Eigen::VectorXd coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * y);

  return coeffs;
}

inline double ApplyCubicPoly(const double x, const Eigen::VectorXd &coeff) {
  if (coeff.rows() != 4) {
    std::cout << "Cubic Coeff Wrong\n";
  }

  Eigen::VectorXd multiplier = Eigen::VectorXd::Ones(4);
  multiplier[0] = 1.0;
  multiplier[1] = x;
  multiplier[2] = x * x;
  multiplier[3] = x * x * x;

  return multiplier.dot(coeff);
}
}  // namespace mono_lane_mapping
