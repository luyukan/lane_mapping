#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/AdaptAutoDiff.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>

#include "catmull_rom_spline.h"

namespace mono_lane_mapping {
class PointSplineFactor
    : public gtsam::NoiseModelFactor5<gtsam::Point3, gtsam::Point3, gtsam::Point3,
                                   gtsam::Point3, gtsam::Pose3> {
 public:
  PointSplineFactor(const gtsam::Key key_ctrl0, const gtsam::Key key_ctrl1,
                    const gtsam::Key key_ctrl2, const gtsam::Key key_ctrl3,
                    const gtsam::Key key_pose,
                    const Eigen::Vector3d &measurement, const double u,
                    const gtsam::SharedNoiseModel &noiseModel)
      : mea_(measurement),
        u_(u), gtsam::NoiseModelFactor5<gtsam::Point3, gtsam::Point3, gtsam::Point3,
                              gtsam::Point3, gtsam::Pose3>(
            noiseModel, key_ctrl0, key_ctrl1, key_ctrl2, key_ctrl3, key_pose) {}

  gtsam::Vector evaluateError(
      const gtsam::Point3 &ctrl0, const gtsam::Point3 &ctrl1,
      const gtsam::Point3 &ctrl2, const gtsam::Point3 &ctrl3,
      const gtsam::Pose3 &pose, gtsam::OptionalMatrixType H_ctrl0,
      gtsam::OptionalMatrixType H_ctrl1, gtsam::OptionalMatrixType H_ctrl2,
      gtsam::OptionalMatrixType H_ctrl3,
      gtsam::OptionalMatrixType H_pose) const override {
    Eigen::MatrixXd spline_mat = Eigen::MatrixXd::Zero(4, 3);
    spline_mat.row(0) = ctrl0.head(3).transpose();
    spline_mat.row(1) = ctrl1.head(3).transpose();
    spline_mat.row(2) = ctrl2.head(3).transpose();
    spline_mat.row(3) = ctrl3.head(3).transpose();

    CatmullRomSpline spline(spline_mat);
    Eigen::Vector3d est_pt = spline.GetPoint(u_).head(3);
    Eigen::Vector4d coeff = spline.GetCoeff(u_).head(4);

    Eigen::Matrix3d Rwb = pose.rotation().matrix();
    Eigen::Vector3d twb = pose.translation();

    Eigen::Vector3d pt_w = Rwb * mea_ + twb;

    gtsam::Vector3 error = est_pt - pt_w;

    if (H_pose) {
      *H_pose = Eigen::MatrixXd::Zero(3, 6);
    }

    if (H_ctrl0) {
      *H_ctrl0 = -Eigen::Matrix3d::Identity() * coeff[0];
      
    }

    if (H_ctrl1) {
      *H_ctrl1 = -Eigen::Matrix3d::Identity() * coeff[1];
    }

    if (H_ctrl2) {
      *H_ctrl2 = -Eigen::Matrix3d::Identity() * coeff[2];
    }

    if (H_ctrl3) {
      *H_ctrl3 = -Eigen::Matrix3d::Identity() * coeff[3];
    }
    // std::cout << "error: " << error.transpose() << std::endl;
    return error;
  }

  

 private:
  Eigen::Vector3d mea_;
  double u_;
};

}  // namespace mono_lane_mapping