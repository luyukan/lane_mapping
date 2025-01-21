#include "gtsam_solver.h"

namespace mono_lane_mapping {

GTSAMSolver& GTSAMSolver::GetInstance() {
  static GTSAMSolver instance;
  return instance;
}

GTSAMSolver::GTSAMSolver() {}

void GTSAMSolver::Init() {}

void GTSAMSolver::reset() {}

void GTSAMSolver::PerformBundleAjustment() {
  gtsam::NonlinearFactorGraph graph;
  auto& sw = SlidingWindow::GetInstance();
  auto& map = MapGraph::GetInstance();

  std::map<int, LandMarkTrackingInfo> lm_tracking_info =
      sw.GetLandmarkTrackingInfo();

  const int min_track_times = 4;

  // store all optimization parameters in vector and reset them after
  // optimization
  std::map<int, std::map<int, std::pair<int, Eigen::Vector3d>>>
      opt_ctrl_pts;  // <lm_id, <ctrl_pt_id, <gtsam_key, position>>>

  std::map<int, std::pair<int, int>>
      gtsam_opt_ctrl_pts;  // <gtsam_key <lm_id, ctrl_pt_id>> for revert
                           // searching
  std::map<int, std::pair<int, Odometry>>
      opt_poses;                       // <window_id <gtsam_key, pose>>
  std::map<int, int> gtsam_opt_poses;  // <gtsam_key, window_id>

  gtsam::Values initial_estimate;
  initial_estimate.clear();

  int gtsam_key = 0;
  for (int i = 0; i < sw.GetCurrentWindowSize(); ++i) {
    Odometry pose = sw.GetWindowPose(i);
    opt_poses[i] = std::make_pair(gtsam_key, pose);
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.topLeftCorner(3, 3) = pose.qwb.toRotationMatrix();
    transformation.topRightCorner(3, 1) = pose.twb;
    gtsam::Pose3 gtsam_pose(transformation);
    initial_estimate.insert_or_assign(gtsam_key, gtsam_pose);
    ++gtsam_key;
  }

  for (int i = 0; i < sw.GetCurrentWindowSize() - 1; ++i) {
    Odometry pose0 = sw.GetWindowPose(i);
    Odometry pose1 = sw.GetWindowPose(i + 1);
    Eigen::Matrix4d transformation0 = Eigen::Matrix4d::Identity();
    transformation0.topLeftCorner(3, 3) = pose0.qwb.toRotationMatrix();
    transformation0.topRightCorner(3, 1) = pose0.twb;
    Eigen::Matrix4d transformation1 = Eigen::Matrix4d::Identity();
    transformation1.topLeftCorner(3, 3) = pose1.qwb.toRotationMatrix();
    transformation1.topRightCorner(3, 1) = pose1.twb;
    Eigen::Matrix4d transformation10 =
        transformation1.inverse() * transformation0;
    gtsam::Pose3 relative_pose_mea(transformation10);
    auto relative_pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05).finished());
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        opt_poses.at(i).first, opt_poses.at(i + 1).first, relative_pose_mea,
        relative_pose_noise));
  }

  gtsam::SharedNoiseModel point_spline_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  for (auto it = lm_tracking_info.begin(); it != lm_tracking_info.end(); ++it) {
    int lm_id = it->first;
    if (it->second.tracking_times < min_track_times) {
      continue;
    }
    LaneLandmark::Ptr lane = map.GetLandmark(lm_id);
    const auto& ctrl_pts = lane->GetControlPoints();
    for (size_t i = 0; i < it->second.window_observations.size(); ++i) {
      int window_id = it->second.window_pose_id.at(i);
      for (size_t j = 0;
           j < it->second.window_observations.at(i).lane_points.size(); ++j) {
        Odometry pose = sw.GetWindowPose(window_id);
        std::vector<int> ctrl_pts_id;
        double pt_u;
        bool add_to_problem = lane->FindFootPoint(
            it->second.window_observations.at(i).lane_points.at(j), pose, lane,
            ctrl_pts_id, pt_u);
        if (!add_to_problem) {
          continue;
        }
        std::vector<int> gtsam_key_vector;
        for (size_t k = 0; k < ctrl_pts_id.size(); ++k) {
          Eigen::Vector3d position =
              lane->GetControlPoint(ctrl_pts_id.at(k)).position;
          gtsam::Point3 gtsam_point(position);
          int key;
          if (opt_ctrl_pts.count(lm_id) &&
              opt_ctrl_pts.at(lm_id).count(ctrl_pts_id[k])) {
            key = opt_ctrl_pts.at(lm_id).at(ctrl_pts_id.at(k)).first;
          } else {
            key = gtsam_key;
            opt_ctrl_pts[lm_id][ctrl_pts_id.at(k)] = std::make_pair(key, position);
            gtsam_opt_ctrl_pts[key] = {lm_id, ctrl_pts_id.at(k)};
            ++gtsam_key;
          }
          initial_estimate.insert_or_assign(key, gtsam_point);
          gtsam_key_vector.push_back(key);
        }
        int pose_gtsam_key = opt_poses.at(window_id).first;
        PointSplineFactor factor(
            gtsam_key_vector.at(0), gtsam_key_vector.at(1),
            gtsam_key_vector.at(2), gtsam_key_vector.at(3), pose_gtsam_key,
            it->second.window_observations.at(i).lane_points.at(j).position,
            pt_u, point_spline_noise);
        graph.add(factor);
      }
    }
  }

  gtsam::LevenbergMarquardtParams params;
  // params.verbosityLM =
  //     gtsam::LevenbergMarquardtParams::VerbosityLM::SUMMARY;  //
  // 输出摘要信息
  // params.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::VALUES;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, params);
  // graph.print("Factor Graph: ");
  // gtsam::Ordering ordering = gtsam::Ordering::Colamd(graph);
  // ordering.print("Variable Ordering: ");
  gtsam::Values result = optimizer.optimize();

  for (auto it = result.begin(); it != result.end(); ++it) {
    gtsam::Key gtsam_key = it->key;
    if (gtsam_key >= sw.GetCurrentWindowSize()) {
      // control point
      int lm_id = gtsam_opt_ctrl_pts.at(gtsam_key).first;
      int ctrl_id = gtsam_opt_ctrl_pts.at(gtsam_key).second;
      gtsam::Point3 optimized_ctrl = result.at<gtsam::Point3>(gtsam_key);
      Eigen::Vector3d new_position = optimized_ctrl.head(3);
      std::cout << "++++++\n";
      std::cout << "Optimized ctrl: " << optimized_ctrl.head(3).transpose()
                << std::endl;
      std::cout << "Origin ctrl: "
                << map.GetLandmark(lm_id)
                       ->GetControlPoint(ctrl_id)
                       .position.transpose()
                << std::endl;

    } else {
    }
  }
}
}  // namespace mono_lane_mapping