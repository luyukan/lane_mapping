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

  last_frame_observation_.timestamp = -1;
  const auto& lane_mapping_parameters = SystemParam::GetInstance().GetLaneMappingParameters();
  candidate_angle_thresh_ = lane_mapping_parameters.candidate_angle_thresh;

  printSlogan();
}

void LaneMapper::InputSyncData(const Odometry &pose,
                               const FrameObservation &frame_observation) {
  FrameObservation cur_frame_observation;
  cur_frame_observation.timestamp = frame_observation.timestamp;
  preprocess_lane_points(frame_observation, cur_frame_observation);

  LaneTracker &lane_tracker = LaneTracker::GetInstance();
  if (initialized_) {
    std::vector<MatchResult> association = lane_tracker.AssociateDetectionWithLast();
  } else {
    if (last_frame_observation_.timestamp > 0) {
      init_map_graph(cur_frame_observation, pose);
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

void LaneMapper::init_map_graph(const FrameObservation &frame_observation, const Odometry &pose) {
  MapGraph &map_graph = MapGraph::GetInstance();

  if (map_graph.GetLandmarks().empty()) {
    for (size_t i = 0; i < frame_observation.lane_observations.size(); ++i) {
      LaneLandmark::Ptr lane_landmark = std::make_shared<LaneLandmark>();
      uint64_t landmark_id = map_graph.GetLaneLandmarkNum();
      lane_landmark->SetId(landmark_id);
      lane_landmark->SetCategory(frame_observation.lane_observations.at(i).category);
      lane_landmark->InitCtrlPointsWithLaneObservation(frame_observation.lane_observations.at(i), pose);
    }

  }
  else {

  }

}

void LaneMapper::printSlogan() {
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

  std::cout << "                                                              " << std::endl;
  std::cout << "##############################################################" << std::endl;
  std::cout << "#                                                            #" << std::endl;
  std::cout << "#  lane_mapping is C++ Implementation of                     #" << std::endl;
  std::cout << "#  HKUST MonoLaneMapping Paper                               #" << std::endl;
  std::cout << "#                                                            #" << std::endl;
  std::cout << "##############################################################" << std::endl;
  std::cout << "                                                              " << std::endl;
}

}  // namespace mono_lane_mapping
