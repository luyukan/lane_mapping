#include "lane_mapper.h"

namespace mono_lane_mapping {

LaneMapper &LaneMapper::GetInstance() {
  static LaneMapper instance;
  return instance;
}
LaneMapper::LaneMapper() {}

void LaneMapper::Init(const std::string &config_file) {
  LanePreprocessor &lane_preprocessor = LanePreprocessor::GetInstance();
  lane_preprocessor.Init(config_file);

  MapGraph &map_graph = MapGraph::GetInstance();
  map_graph.Init();

  LaneTracker &lane_tracker = LaneTracker::GetInstance();
  lane_tracker.Init();

  last_frame_observation_.timestamp = -1;
}

void LaneMapper::InputSyncData(const Odometry &pose,
                               const FrameObservation &frame_observation) {
  FrameObservation cur_frame_observation;
  cur_frame_observation.timestamp = frame_observation.timestamp;
  preprocess_lane_points(frame_observation, cur_frame_observation);

  LaneTracker &lane_tracker = LaneTracker::GetInstance();
  if(initialized_) {
    std::vector<MatchResult> association = lane_tracker.AssociateDetectionWithLast();
  }
  else {
    if (last_frame_observation_.timestamp > 0) {
      init_map_graph(frame_observation);
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

void LaneMapper::init_map_graph(const FrameObservation &frame_observation) {
  MapGraph &map_graph = MapGraph::GetInstance();

  LaneLandmark::Ptr lane_landmark = std::make_shared<LaneLandmark>();
  uint64_t landmark_id = map_graph.GetLaneLandmarkNum();
  lane_landmark->SetId(landmark_id);

}

}  // namespace mono_lane_mapping
