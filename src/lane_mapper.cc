#include "lane_mapper.h"

namespace mono_lane_mapping {

LaneMapper &LaneMapper::GetInstance() {
  static LaneMapper instance;
  return instance;
}
LaneMapper::LaneMapper() {}

void LaneMapper::Init() {
  LanePreprocessor &lane_preprocessor = LanePreprocessor::GetInstance();
  lane_preprocessor.Init();

  MapGraph &map_graph = MapGraph::GetInstance();
  map_graph.Init();

  LaneTracker &lane_tracker = LaneTracker::GetInstance();
  lane_tracker.Init();

  SlidingWindow &sliding_w = SlidingWindow::GetInstance();
  sliding_w.Init();

  last_frame_observation_.timestamp = -1;
  const auto &lane_mapping_parameters =
      SystemParam::GetInstance().GetLaneMappingParameters();
  candidate_angle_thresh_ = lane_mapping_parameters.candidate_angle_thresh;

  printInformation();
}

void LaneMapper::InputSyncData(const Odometry &pose,
                               const FrameObservation &frame_observation) {
  FrameObservation cur_frame_observation;
  cur_frame_observation.timestamp = frame_observation.timestamp;
  preprocess_lane_points(frame_observation, cur_frame_observation);

  LaneTracker &lane_tracker = LaneTracker::GetInstance();
  if (initialized_) {
    track_with_map(frame_observation, pose);
    smooth();
  } else {
    if (last_frame_observation_.timestamp > 0) {
      init_map(cur_frame_observation, pose);
      last_frame_observation_ = frame_observation;
      initialized_ = true;
    }
  }
  last_frame_observation_ = frame_observation;

}

void LaneMapper::preprocess_lane_points(
    const FrameObservation &frame_observation,
    FrameObservation &cur_frame_observation) {
  LanePreprocessor &lane_preprocessor = LanePreprocessor::GetInstance();
  lane_preprocessor.DenoiseLanePoints(frame_observation, cur_frame_observation);
}

void LaneMapper::track_with_last_frame(
    const FrameObservation &frame_observation, const Odometry &pose) {}

void LaneMapper::track_with_map(const FrameObservation &frame_observation,
                                const Odometry &pose) {
  auto &tracker = LaneTracker::GetInstance();
  std::vector<MatchResult> matching_res =
      tracker.TrackWithMap(frame_observation, pose);

  // init landmark if untracked
  MapGraph &map_graph = MapGraph::GetInstance();
  for (size_t i = 0; i < matching_res.size(); ++i) {
    if (matching_res.at(i).trainIdx == -1) {
      // initialize landmark
      LaneLandmark::Ptr lane_landmark = std::make_shared<LaneLandmark>();
      int landmark_id = map_graph.GetLaneLandmarkNum();
      lane_landmark->SetId(landmark_id);
      lane_landmark->SetCategory(
          frame_observation.lane_observations.at(i).category);
      map_graph.AddLandmark(lane_landmark);
      matching_res.at(i).trainIdx = landmark_id;
    } else {
      int landmark_id = matching_res.at(i).trainIdx;
      auto landmark = map_graph.GetLandmark(landmark_id);
      landmark->UpdateCtrlPointsWithLaneObservation(
          frame_observation.lane_observations.at(i), pose);
    }
  }

  // update information in sliding window
  auto &sl_window = SlidingWindow::GetInstance();
  sl_window.UpdateWindowStatus(frame_observation, pose, matching_res);
}

void LaneMapper::init_map(const FrameObservation &frame_observation,
                          const Odometry &pose) {
  MapGraph &map_graph = MapGraph::GetInstance();

  for (size_t i = 0; i < frame_observation.lane_observations.size(); ++i) {
    LaneLandmark::Ptr lane_landmark = std::make_shared<LaneLandmark>();
    int landmark_id = map_graph.GetLaneLandmarkNum();
    lane_landmark->SetId(landmark_id);
    lane_landmark->SetCategory(
        frame_observation.lane_observations.at(i).category);
    lane_landmark->InitCtrlPointsWithLaneObservation(
        frame_observation.lane_observations.at(i), pose);
    map_graph.AddLandmark(lane_landmark);
  }
}

void LaneMapper::smooth() {
  GTSAMSolver &solver = GTSAMSolver::GetInstance();
  solver.PerformBundleAjustment();
}

void LaneMapper::printInformation() {
  std::cout << R"(
          _oo0oo_
         o8888888o
         88" . "88
         (| -_- |)
         0\  =  /0
       ___/`---'\___
     .' \\|     |// '.
    / \\|||  :  |||// \
   / _||||| -:- |||||- \
  |   | \\\  -  /// |   |
  | \_|  ''\---/''  |_/ |
  \  .-\__  '-'  ___/-. /
___'. .'  /--.--\  `. .'___
."" '<  `.___\_<|>_/___.' >' "".
| | :  `- \`.;`\ _ /`;.`/ - ` : | |
\  \ `_.   \_ __\ /__ _/   .-` /  /
====`-.____`.___ \_____/__.-`____.-'====
             `=---='

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
          佛祖保佑       永无BUG
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    )" << std::endl;

  std::cout << "                                                              "
            << std::endl;
  std::cout << "##############################################################"
            << std::endl;
  std::cout << "#                                                            #"
            << std::endl;
  std::cout << "#  lane_mapping is C++ Implementation of                     #"
            << std::endl;
  std::cout << "#  HKUST MonoLaneMapping Paper                               #"
            << std::endl;
  std::cout << "#                                                            #"
            << std::endl;
  std::cout << "##############################################################"
            << std::endl;
  std::cout << "                                                              "
            << std::endl;
}

}  // namespace mono_lane_mapping
