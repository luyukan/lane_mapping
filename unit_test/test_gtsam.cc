#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Eigen>
#include <iostream>
#include <random>

class LidarReprojectionFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  LidarReprojectionFactor(const gtsam::Key key_pose, const Eigen::Vector3d &mea,
                          const Eigen::Vector3d &pt_w,
                          const gtsam::SharedNoiseModel &noiseModel)
      : mea_(mea),
        pt_w_(pt_w),
        gtsam::NoiseModelFactor1<gtsam::Pose3>(noiseModel, key_pose) {}

  gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                              gtsam::OptionalMatrixType H) const override {
    // gtsam::Pose3 inv_pose = pose.inverse();
    gtsam::Point3 pt_lidar = pose.transformTo(pt_w_, H);
    gtsam::Vector3 error = pt_lidar - mea_;

    return error;
  }

 private:
  Eigen::Vector3d mea_;
  Eigen::Vector3d pt_w_;
};

int main() {
  std::random_device rd;  // Random seed from hardware
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> distri_point_range(20.0, 40.0);
  std::uniform_real_distribution<> distri_point_translation(20.0, 40.0);
  std::uniform_real_distribution<> distri_point_axang(-3.0, 3.0);

  int num = 100;
  std::vector<Eigen::Vector3d> landmarks;
  for (size_t i = 0; i < num; ++i) {
    Eigen::Vector3d point = Eigen::Vector3d::Random() * distri_point_range(gen);
    landmarks.push_back(point);
  }

  Eigen::Vector3d twb = Eigen::Vector3d::Random() * 5.0;
  Eigen::Vector3d axang_wb =
      Eigen::Vector3d::Random() * distri_point_axang(gen);
  Eigen::Matrix3d Rwb =
      Eigen::AngleAxisd(axang_wb.norm(), axang_wb.normalized())
          .toRotationMatrix();

  std::vector<Eigen::Vector3d> measurements;
  for (size_t i = 0; i < landmarks.size(); ++i) {
    Eigen::Matrix3d Rbw = Rwb.transpose();
    Eigen::Vector3d tbw = -Rwb.transpose() * twb;
    measurements.push_back(Rbw * landmarks.at(i) + tbw);
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initialEstimate;
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3::Ones() * 0.1);  // 0.1 in all 3 dimensions

  // Define the measurement (expec

  initialEstimate.insert(
      0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)));

  for (size_t i = 0; i < landmarks.size(); ++i) {
    LidarReprojectionFactor factor(0, measurements.at(i), landmarks.at(i),
                                   noise);
    graph.add(factor);
  }

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  gtsam::Values result = optimizer.optimize();

  std::cout << "Original translation: \n" << twb.transpose() << std::endl;
  std::cout << "Original rotation: \n" << Rwb << std::endl;

  std::cout << "Optimized translation: \n"
            << result.at<gtsam::Pose3>(0).translation() << std::endl;
  std::cout << "Optimized rotation: \n"
            << result.at<gtsam::Pose3>(0).rotation() << std::endl;

  return 0;
}