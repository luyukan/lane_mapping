//
// Created by yukan on 25-1-20.
//

#include "map_graph.h"

namespace mono_lane_mapping {
MapGraph::MapGraph() {}

MapGraph &MapGraph::GetInstance() {
  static MapGraph instance;
  return instance;
}
void MapGraph::Init() {}

const std::map<uint64_t, LaneLandmark::Ptr>& MapGraph::GetLandmarks() {
  return lane_landmarks_;
}

uint64_t MapGraph::GetLaneLandmarkNum() {
  return lane_landmarks_.size();
}
}  // namespace mono_lane_mapping
