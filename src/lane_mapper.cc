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
}

void LaneMapper::InputSyncData(const Odometry &pose,
                               const FrameObservation &frame_observation) {
  FrameObservation cur_frame_observation;
  cur_frame_observation.timestamp = frame_observation.timestamp;
  preprocess_lane_points(frame_observation, cur_frame_observation);


}

void LaneMapper::preprocess_lane_points(
    const FrameObservation &frame_observation,
    FrameObservation &cur_frame_observation) {
  LanePreprocessor &lane_preprocessor = LanePreprocessor::GetInstance();
  lane_preprocessor.DenoiseLanePoints(frame_observation, cur_frame_observation);
}

}  // namespace mono_lane_mapping
