#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Dense>
#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

using namespace gtsam;
using symbol_shorthand::L;
using symbol_shorthand::X;

class LaneOptimizer : public LaneTracker {
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

 public:
  LaneOptimizer(const std::string& bagFile) : LaneTracker(bagFile) {
    // 初始化参数
    windowSize = cfg::lane_mapping.window_size;
    use_isam = true;

    // ISAM2配置
    ISAM2Params parameters;
    parameters.setRelinearizeThreshold(0.0);
    parameters.relinearizeSkip = 0;
    parameters.enableRelinearization = false;
    isam = ISAM2(parameters);
  }

  void add_keyframe() {
    if (use_isam) {
      sliding_window = {cur_frame};
      return;
    }

    if (sliding_window.size() < window_size + 1) {
      sliding_window.push_back(cur_frame);
      return;
    } else {
      // Extract the second and third latest frames
      Frame* second_latest_frame = sliding_window[sliding_window.size() - 2];
      Frame* third_latest_frame = sliding_window[sliding_window.size() - 3];

      // Compute delta pose and its components (translation and rotation)
      Eigen::Matrix4d delta_pose =
          third_latest_frame->T_cw * second_latest_frame->T_wc;
      double delta_xyz = delta_pose.block<3, 1>(0, 3).norm();
      double delta_angle = rot_to_angle(
          delta_pose.block<3, 3>(0, 0));  // Assume rot_to_angle is implemented

      // Thresholds to decide whether to use the latest frame or discard it
      if (delta_xyz > 3.0 || delta_angle > 10.0) {
        margin_old = true;
      } else {
        margin_old = false;
      }

      sliding_window.back() =
          cur_frame;  // Replace the oldest frame with the current one
    }
  }

  gtsam::SharedNoiseModel get_pt_noise_model(double noise, bool huber = false,
                                             int dim = 3,
                                             double huber_thresh = 1.0) {
    auto ctrlpt_noise_model = gtsam::noiseModel::Isotropic::Sigma(dim, noise);
    if (huber) {
      ctrlpt_noise_model = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(huber_thresh),
          ctrlpt_noise_model);
    }
    return ctrlpt_noise_model;
  }

  bool set_gtsam_symbol(int lane_id, int ctrl_pt_id, Node* node) {
    auto key = std::make_pair(lane_id, ctrl_pt_id);
    if (lanes_in_graph.find(key) != lanes_in_graph.end()) {
      return true;
    } else {
      int idx = lanes_in_graph.size();
      lanes_in_graph[key] = idx;
      key_in_graph[idx] = node;
      node->set_key(idx);
      node->set_lane_id(lane_id);
      return false;
    }
  }

  void create_new_lane() {
    // Iterate over the lane features in the current frame
    for (const auto& lf : cur_frame.get_lane_features()) {
      if (lf.id == -1) {
        continue;
      }

      // Transform the lane feature to world coordinates
      LaneFeature lane_feature_w = cur_frame.transform_to_world(lf);

      // Fit the lane feature
      lane_feature_w.fitting();

      // Check if the lane is not already in the map
      if (lanes_in_map.find(lane_feature_w.id) == lanes_in_map.end()) {
        // Add new lane and build the KDTree
        lanes_in_map[lane_feature_w.id] = lane_feature_w;

        // Initialize control points for the new lane
        lanes_in_map[lane_feature_w.id].init_ctrl_pts(lane_feature_w,
                                                      cur_frame.T_cw);
      } else {
        // If the lane is already in the map, update its control points
        lanes_in_map[lane_feature_w.id].update_ctrl_pts(lane_feature_w);
      }
    }
  }

  gtsam::NonlinearFactor::shared_ptr lane_factor(
      const gtsam::Point3& pt_w, const gtsam::Point3& pt_c, double u,
      const std::vector<std::shared_ptr<Node>>& ctrl_pts, double noise,
      int frame_id) {
    // 获取噪声模型，使用 Huber 损失
    auto noise_model = get_pt_noise_model(noise, true, 3, 0.5);

    // 生成 GTSAM CustomFactor
    std::vector<gtsam::Key> keys = {
        ctrl_pts[0]->get_key(), ctrl_pts[1]->get_key(), ctrl_pts[2]->get_key(),
        ctrl_pts[3]->get_key()};

    // 绑定误差函数（类似 Python `partial(error_catmull_rom, pt_w)`)
    auto factor_func = [pt_w](const gtsam::Values& values,
                              gtsam::Vector& residual) {
      return error_catmull_rom(values, residual, pt_w);
    };

    return boost::make_shared<gtsam::CustomFactor>(noise_model, keys,
                                                   factor_func);
  }

  gtsam::Vector error_catmull_rom(
      const gtsam::Values& values, const gtsam::Vector& measurement,
      boost::optional<std::vector<Eigen::MatrixXd>&> jacobians = boost::none) {
    // 读取控制点
    std::vector<gtsam::Point3> ctrl_pts;
    for (size_t i = 0; i < 4; ++i) {
      ctrl_pts.push_back(values.at<gtsam::Point3>(i));
    }

    // 创建 Catmull-Rom Spline
    CatmullRomSpline spline(ctrl_pts);

    // 获取 u 和 估计点
    double u = measurement(3);
    auto [est_pt, coeff] = spline.get_point(u);

    // 计算误差
    gtsam::Vector3 error =
        est_pt - gtsam::Point3(measurement(0), measurement(1), measurement(2));

    // 计算 Jacobian
    if (jacobians) {
      for (size_t i = 0; i < 4; ++i) {
        (*jacobians)[i] = Eigen::Matrix3d::Identity() * coeff(i);
      }
    }

    return error;
  }

  void update_key_status(
      const std::vector<std::shared_ptr<Node>>& ctrl_pts,
      std::unordered_map<int, std::vector<int>>& key_status) {
    // 遍历控制点，更新 key_status
    for (const auto& ctrl_pt : ctrl_pts) {
      gtsam::Key key = ctrl_pt->get_key();

      if (key_status.find(key) == key_status.end()) {
        key_status[key] = {1, 0};  // 初次观测，初始化为 {1, 0}
      } else {
        key_status[key][0] += 1;  // 增加观测次数
      }
    }

    // 更新主观测次数
    key_status[ctrl_pts[1]->get_key()][1] += 1;
    key_status[ctrl_pts[2]->get_key()][1] += 1;
  }

  void add_chordal_factor(
      std::unordered_map<int, std::vector<int>>& key_status_cur) {
    std::unordered_map<std::pair<gtsam::Key, gtsam::Key>, bool, pair_hash>
        chordal_factors_cur;

    std::vector<gtsam::Key> constrained_keys = graph.keyVector();

    for (const auto& key : constrained_keys) {
      if (key_in_graph.find(key) == key_in_graph.end()) {
        continue;
      }

      auto node = key_in_graph[key];
      int lane_id = node->get_lane_id();

      if (lanes_in_map[lane_id].size() < 2) {
        continue;
      }

      int idx = lanes_in_map[lane_id].get_ctrl_pt_idx(node);
      gtsam::Key adj_key;
      std::pair<gtsam::Key, gtsam::Key> key_pair;

      if (idx != 0) {
        adj_key = lanes_in_map[lane_id].get_ctrl_node(idx - 1)->get_key();
        key_pair = {adj_key, key};
      } else {
        adj_key = lanes_in_map[lane_id].get_ctrl_node(idx + 1)->get_key();
        key_pair = {key, adj_key};
      }

      if (adj_key != 0 &&
          std::find(constrained_keys.begin(), constrained_keys.end(),
                    adj_key) != constrained_keys.end()) {
        chordal_factors_cur[key_pair] = true;
      }

      if (idx != lanes_in_map[lane_id].size() - 1) {
        adj_key = lanes_in_map[lane_id].get_ctrl_node(idx + 1)->get_key();
        key_pair = {key, adj_key};
      }

      if (adj_key != 0 &&
          std::find(constrained_keys.begin(), constrained_keys.end(),
                    adj_key) != constrained_keys.end()) {
        chordal_factors_cur[key_pair] = true;
      }
    }

    // 合并到 chordal_factors
    for (const auto& [key_pair, _] : chordal_factors_cur) {
      if (chordal_factors.find(key_pair) == chordal_factors.end()) {
        chordal_factors[key_pair] = true;

        // 获取噪声模型
        auto noise_model = get_pt_noise_model(1.0, false, 3);

        // 计算测量值
        gtsam::Point3 meas = key_in_graph[key_pair.second]->item() -
                             key_in_graph[key_pair.first]->item();

        // 添加因子
        graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::Point3>>(
            key_pair.first, key_pair.second, meas, noise_model));
      }
    }
  }

  void add_ctrl_factor(std::unordered_map<gtsam::Key, std::vector<int>>&
                           key_status const gtsam::Vector3& ctrl_noise) {
    std::vector<gtsam::Key> constrained_keys;

    // 如果使用 iSAM，则获取 graph 中的 keys
    if (use_isam) {
      constrained_keys = graph.keyVector();
    }

    std::vector<gtsam::Key> key_unstable;

    // 过滤不稳定的关键点
    for (const auto& [key, obs] : key_status) {
      if (obs[0] < 4 || obs[1] < 1) {
        key_unstable.push_back(key);
      }
    }

    // 处理不稳定的关键点
    for (const auto& key : key_unstable) {
      auto node = key_in_graph[key];

      // 更新初始估计
      initial_estimate.insert_or_assign(key, gtsam::Point3(node->item()));

      // 复制噪声模型
      gtsam::SharedNoiseModel ctrlpt_noise_model =
          gtsam::noiseModel::Diagonal::Sigmas(ctrl_noise);

      // 添加先验因子
      graph.add(boost::make_shared<gtsam::PriorFactor<gtsam::Point3>>(
          key, gtsam::Point3(node->item()), ctrlpt_noise_model));
    }
  }

  void buildGraph() {
    graph.clear();
    initial_estimate.clear();
    lane_meas.clear();

    if (!cfg.lane_mapping.init_after_opt) {
      create_new_lane();
    }

    // Record the landmarks in the sliding window
    std::set<int> lm_in_window;
    for (const auto& frame : sliding_window) {
      for (const auto& lf : frame.get_lane_features()) {
        if (lf.id != -1 && lanes_in_map.find(lf.id) != lanes_in_map.end()) {
          lm_in_window.insert(lf.id);
        }
      }
    }

    // Prepare the grid for downsample
    for (const int lm_id : lm_in_window) {
      lanes_in_map[lm_id].ctrl_pts.update_kdtree();
      lane_grid[lm_id] = {};
      for (const auto& ctrl_pt :
           (use_isam ? std::vector<CtrlPt>{}
                     : lanes_in_map[lm_id].get_ctrl_nodes())) {
        lane_grid[lm_id][ctrl_pt.id] = {};
        for (int j = 0; j < cfg.lane_mapping.lane_sample_num; ++j) {
          lane_grid[lm_id][ctrl_pt.id].push_back(
              {std::numeric_limits<double>::infinity()});
        }
      }
    }

    std::vector<std::vector<double>> pts_cp;

    for (const auto& frame : sliding_window) {
      for (size_t i = 0; i < frame.get_lane_features().size(); ++i) {
        const auto& lf = frame.get_lane_features()[i];
        if (lf.id == -1 || lanes_in_map.find(lf.id) == lanes_in_map.end()) {
          continue;
        }

        int lm_id = lf.id;
        auto& lm = lanes_in_map[lm_id];
        for (size_t j = 0; j < lf.get_xyzs().size(); ++j) {
          auto pt_c = lf.get_xyzs()[j];
          auto pt_w = frame.T_wc.block<3, 3>(0, 0) * pt_c.head<3>() +
                      frame.T_wc.col(3).head<3>();
          auto [ctrl_pts, u, error] = lm.ctrl_pts.find_footpoint(pt_w);
          //  大概估计一下误差，如果误差太大，就不要加入因子了
          if (ctrl_pts == nullptr || error > 10.0 || u < 0 || u > 1) {
            continue;
          }

          pt_w.conservativeResize(4);
          pt_w[3] = u;

          if (use_isam) {
            pts_cp.push_back(
                {pt_w, pt_c, ctrl_pts, lm_id, frame.frame_id, lf.noise[j]});
          } else {
            int interval_id = std::clamp(
                static_cast<int>(u * cfg.lane_mapping.lane_sample_num), 0,
                cfg.lane_mapping.lane_sample_num - 1);
            if (lf.noise[j] <
                lane_grid[lm_id][ctrl_pts[1].id][interval_id].back()) {
              lane_grid[lm_id][ctrl_pts[1].id][interval_id] = {
                  pt_w, pt_c, ctrl_pts, lm_id, frame.frame_id, lf.noise[j]};
            }
          }
        }

        lanes_in_map[lm_id].update_raw_pts(frame.transform_to_world(lf),
                                           frame.frame_id);

        if (debug_flag) {
          lane_meas.push_back(frame.transform_to_world(lf));
        }
      }
    }

    for (const int lm_id : (use_isam ? std::vector<int>{} : lm_in_window)) {
      for (const auto& ctrl_pt_id : lane_grid[lm_id]) {
        for (int interval_id = 0;
             interval_id < cfg.lane_mapping.lane_sample_num; ++interval_id) {
          if (lane_grid[lm_id][ctrl_pt_id.first][interval_id].back() <
              std::numeric_limits<double>::infinity()) {
            pts_cp.push_back(lane_grid[lm_id][ctrl_pt_id.first][interval_id]);
          }
        }
      }
    }

    std::unordered_map<int, std::vector<int>> key_status_cur;

    for (const auto& pt_data : pts_cp) {
      auto [pt_w, pt_c, ctrl_pts, lm_id, frame_id, noise] = pt_data;

      // Insert initial estimate
      for (const auto& ctrl_pt : ctrl_pts) {
        bool in_graph = set_gtsam_symbol(lm_id, ctrl_pt.id, ctrl_pt);
        int key = ctrl_pt.get_key();
        if (use_isam) {
          if (!in_graph) {
            initial_estimate.insert_or_assign(key, gtsam::Point3(ctrl_pt.item));
          }
        } else {
          initial_estimate.insert_or_assign(key, gtsam::Point3(ctrl_pt.item));
        }

        // Create a factor and update key status
        auto gf = lane_factor(pt_w, pt_c, pt_w[3], ctrl_pts, noise, frame_id);
        factor_candidates.push_back(
            {gf, pt_w, pt_c, std::vector<double>{}, lm_id, frame_id, noise});
        update_key_status(ctrl_pts, key_status_cur);
      }
    }

    std::vector<std::vector<double>> added_cp_d;

    if (use_isam) {
      int added_num = 0;
      for (auto& [key, status] : key_status_cur) {
        if (key_status.find(key) == key_status.end()) {
          key_status[key] = status;
        } else {
          key_status[key][0] += status[0];
          key_status[key][1] += status[1];
        }
      }

      std::vector<int> remove_idx;
      for (size_t i_gf = 0; i_gf < factor_candidates.size(); ++i_gf) {
        auto& [gf, pt_w, pt_c, ctrl_pts, lm_id, frame_id, noise] =
            factor_candidates[i_gf];
        bool valid = true;
        for (const auto& key : gf.keys()) {
          if (key_in_graph.find(key) == key_in_graph.end()) {
            continue;
          }
          if (key_status[key][0] < 4) {
            valid = false;
            break;
          }
        }

        if (valid) {
          graph.add(gf);
          remove_idx.push_back(i_gf);
          added_num++;
          added_cp_d.push_back(
              {pt_w, pt_c, ctrl_pts, lm_id, frame_id, noise, graph.size() - 1});
        }
      }

      for (auto idx = remove_idx.rbegin(); idx != remove_idx.rend(); ++idx) {
        factor_candidates.erase(factor_candidates.begin() + *idx);
      }

      add_chordal_factor(key_status_cur);
    } else {
      for (const auto& [gf, pt_w, pt_c, ctrl_pts, lm_id, frame_id, noise] :
           factor_candidates) {
        graph.add(gf);
        added_cp_d.push_back(
            {pt_w, pt_c, ctrl_pts, lm_id, frame_id, noise, graph.size() - 1});
      }
      factor_candidates.clear();
      add_ctrl_factor(key_status_cur);
    }

    if (debug_flag) {
      lanes_prev.clear();
      for (const int lm_id : lm_in_window) {
        lanes_prev.push_back(lanes_in_map[lm_id]);
      }
    }

    pts_cp_valid = added_cp_d;
  }

  void map_update() {
    add_keyframe();

    if (add_odo_noise) {
      update_pose();
    }

    build_graph();
    optimization();
    slide_window();

    if (cfg.debug_flag) {
      std::vector<LaneFeature> lanes_opted;
      for (int lane_id : lm_in_window) {
        lanes_opted.push_back(
            lanes_in_map[lane_id]);  // Assuming deep copy is not needed in C++
      }
      visualize_optimization(lanes_prev, lanes_opted, lane_meas, pts_cp_valid);
    }
  }

  void slide_window() {
    if (use_isam) {
      return;
    }

    if (sliding_window.size() < window_size + 1) {
      return;
    }

    // Get the latest frame (last element)
    Frame* latest_frame = sliding_window.back();

    if (margin_old) {
      // Remove the oldest frame (first element) and append the latest frame
      sliding_window.erase(sliding_window.begin());
      sliding_window.push_back(latest_frame);
    } else {
      // Remove the second-to-last frame and insert the latest frame before the
      // last one
      sliding_window.erase(sliding_window.end() - 2);
      sliding_window.insert(sliding_window.end() - 1, latest_frame);
    }
  }

  bool update_pose() {
    // lm_in_window = list(set([lf.id for frame in self.sliding_window for lf in
    // frame.get_lane_features()
    //                           if lf.id != -1 and lf.id in
    //                           self.lanes_in_map]))

    std::set<int> lm_in_window;
    for (const auto& frame : sliding_window) {
      for (const auto& lf : frame.get_lane_features()) {
        if (lf.id != -1 && lanes_in_map.find(lf.id) != lanes_in_map.end()) {
          lm_in_window.insert(lf.id);
        }
      }
    }

    // Update KDTree for each lane feature
    for (int lm_id : lm_in_window) {
      lanes_in_map[lm_id].ctrl_pts.update_kdtree();
    }

    Pose3 last_pose = cur_frame->T_wc;
    for (int i = 0; i < 1; ++i) {
      // Add pose to graph
      NonlinearFactorGraph graph;
      Values initial_estimate;
      Vector odo_noise = cfg.pose_update.odom_noise;
      for (int j = 0; j < 3; ++j) {
        odo_noise[j] = odo_noise[j] * M_PI / 180.0;  // Convert to radians
      }
      noiseModel::Diagonal::shared_ptr odo_noise_model =
          noiseModel::Diagonal::Sigmas(odo_noise);

      if (cur_frame->frame_id == 0) {
        graph.add(PriorFactorPose3(X(cur_frame->frame_id),
                                   Pose3(cur_frame->T_wc), odo_noise_model));
        initial_estimate.insert(X(cur_frame->frame_id), Pose3(cur_frame->T_wc));
        gtsam_key_frame[X(cur_frame->frame_id)] = cur_frame;
      } else {
        graph.add(PriorFactorPose3(X(prev_frame->frame_id),
                                   Pose3(prev_frame->T_wc), odo_noise_model));
        initial_estimate.insert(X(prev_frame->frame_id),
                                Pose3(prev_frame->T_wc));
        graph.add(BetweenFactorPose3(X(prev_frame->frame_id),
                                     X(cur_frame->frame_id), Pose3(odo_meas),
                                     odo_noise_model));
        initial_estimate.insert(X(cur_frame->frame_id), Pose3(cur_frame->T_wc));
        gtsam_key_frame[X(cur_frame->frame_id)] = cur_frame;
        gtsam_key_frame[X(cur_frame->frame_id - 1)] = prev_frame;
      }

      std::vector<Vector3> directions;
      for (size_t i = 0; i < cur_frame->get_lane_features().size(); ++i) {
        const auto& lf = cur_frame->get_lane_features()[i];
        if (lf.id == -1 || lanes_in_map.find(lf.id) == lanes_in_map.end()) {
          continue;
        }

        int lm_id = lf.id;
        auto& lm = lanes_in_map[lm_id];
        for (size_t j = 0; j < lf.get_xyzs().size(); ++j) {
          const Vector3& pt_c = lf.get_xyzs()[j];
          if (pt_c.norm() > cfg.pose_update.max_range) {
            continue;
          }

          // Transform point from camera to world
          Vector3 pt_w = cur_frame->T_wc.rotation() * pt_c.head<3>() +
                         cur_frame->T_wc.translation();

          // Find closest control points and error
          std::vector<LaneFeatureControlPoint> ctrl_pts;
          double u, error;
          lm.ctrl_pts.find_footpoint(pt_w, ctrl_pts, u, error);

          if (ctrl_pts.empty() || error > 2.0) {
            continue;
          }

          Eigen::MatrixXd ctrl_pts_np(ctrl_pts.size(), 3);
          for (size_t k = 0; k < ctrl_pts.size(); ++k) {
            ctrl_pts_np.row(k) = ctrl_pts[k].item.head<3>().transpose();
          }

          double noise = (cfg.pose_update.meas_noise > 0)
                             ? cfg.pose_update.meas_noise
                             : lf.noise[j];
          noiseModel::Base::shared_ptr noise_model;
          if (cfg.pose_update.use_huber) {
            noise_model =
                get_pt_noise_model(noise, true, cfg.pose_update.huber_thresh);
          } else {
            noise_model = get_pt_noise_model(noise, false);
          }

          // Add custom factor
          std::shared_ptr<CustomFactor> gf = std::make_shared<CustomFactor>(
              noise_model, std::vector<Key>{X(cur_frame->frame_id)},
              std::bind(&PosePointTangentFactor, pt_c, u, ctrl_pts_np));
          graph.add(gf);

          // Compute directions using spline
          CatmullRomSpline spline(ctrl_pts_np);
          Vector3 di = spline.get_derivative(u);
          directions.push_back(di);
        }
      }

      // Optimize
      GaussNewtonParams params;
      params.setMaxIterations(5);
      GaussNewtonOptimizer optimizer(graph, initial_estimate, params);
      Values result = optimizer.optimize();

      Key key = X(cur_frame->frame_id);

      if (cfg.pose_update.reproject && !directions.empty()) {
        Matrix4 update_pose =
            inv_se3(cur_frame->T_wc) * result.atPose3(key).matrix();
        Vector3 degeneracy_d = std::accumulate(
            directions.begin(), directions.end(), Vector3::Zero());
        degeneracy_d /= directions.size();
        degeneracy_d = cur_frame->T_wc.rotation().transpose() * degeneracy_d;
        degeneracy_d /= degeneracy_d.norm();

        Vector3 update_xyz = update_pose.block<3, 1>(0, 3);
        Vector3 nex_xyz =
            (Matrix3d::Identity() - degeneracy_d * degeneracy_d.transpose()) *
            update_xyz;
        update_pose.block<3, 1>(0, 3) = nex_xyz;

        cur_frame->T_wc = cur_frame->T_wc * update_pose;
      } else {
        cur_frame->T_wc = result.atPose3(key).matrix();
      }

      Matrix4 delta = inv_se3(last_pose) * cur_frame->T_wc;
      if (rot_to_angle(delta.block<3, 3>(0, 0), true) < 0.1) {
        break;
      }
      last_pose = cur_frame->T_wc;
    }

    return true;
  }

  void optimization() {
    if (use_isam) {
      isam.update(graph, initialEstimate);
      Values result = isam.calculateEstimate();
    } else {
      GaussNewtonParams params;
      params.setMaxIterations(5);
      GaussNewtonOptimizer optimizer(graph, initialEstimate, params);
      Values result = optimizer.optimize();
    }

    // 更新控制点
    for (const auto& key : result.keys()) {
      if (keyInGraph.count(key)) {
        auto node = keyInGraph[key];
        node->setItem(result.at<Point3>(key));
      }
    }
  }
};