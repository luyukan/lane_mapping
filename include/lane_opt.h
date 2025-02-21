#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "lane_tracker.h"

using namespace gtsam;
using symbol_shorthand::L;
using symbol_shorthand::X;

class LaneOptimizer : public LaneTracker {
 public:
  LaneOptimizer();

  void add_keyframe();

  gtsam::SharedNoiseModel get_pt_noise_model(double noise, bool huber = false,
                                             int dim = 3,
                                             double huber_thresh = 1.0);

  bool set_gtsam_symbol(int lane_id, int ctrl_pt_id, Node* node);

  void create_new_lane();

  gtsam::NonlinearFactor::shared_ptr lane_factor(
      const gtsam::Point3& pt_w, const gtsam::Point3& pt_c, double u,
      const std::vector<std::shared_ptr<Node>>& ctrl_pts, double noise,
      int frame_id);

  gtsam::Vector error_catmull_rom(
      const gtsam::Values& values, const gtsam::Vector& measurement,
      boost::optional<std::vector<Eigen::MatrixXd>&> jacobians = boost::none);

  void update_key_status(const std::vector<std::shared_ptr<Node>>& ctrl_pts,
                         std::unordered_map<int, std::vector<int>>& key_status);

  void add_chordal_factor(
      std::unordered_map<int, std::vector<int>>& key_status_cur);

  void add_ctrl_factor(std::unordered_map<gtsam::Key, std::vector<int>>&
                           key_status const gtsam::Vector3& ctrl_noise);

  void buildGraph();

  void map_update();

  void slide_window();

  bool update_pose();

  void optimization();

 private:
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;
  ISAM2 isam;
  std::map<int, std::shared_ptr<LaneFeature>> lanesInMap;
  std::vector<Frame> slidingWindow;
  std::vector<std::vector<std::vector<double>>> pts_cp_valid;
  std::vector<std::vector<std::vector<double>>> factor_candidates;
  std::vector<std::vector<LaneFeature>>
      lane_meas;  // Assuming LaneFeature is a custom class.
  std::unordered_map<int, Lane> lanes_in_map;  // Map of lane ID to lane object
  std::unordered_map<int,
                     std::unordered_map<int, std::vector<std::vector<double>>>>
      lane_grid;
  int windowSize;
  bool use_isam = true;  // Just an example, should be set appropriately
  bool debug_flag = false;

  std::unordered_map<std::pair<int, int>, int> lanes_in_graph;
  std::unordered_map<int, Node*> key_in_graph;

  std::unordered_map<std::pair<gtsam::Key, gtsam::Key>, bool, pair_hash>&
      chordal_factors;
  std::deque<Frame>  sliding_window;
};