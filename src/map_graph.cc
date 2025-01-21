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

const std::map<int, LaneLandmark::Ptr> &MapGraph::GetLandmarks() {
  return lane_landmarks_;
}

int MapGraph::GetLaneLandmarkNum() { return lane_landmarks_.size(); }

void MapGraph::AddLandmark(const LaneLandmark::Ptr &landmark) {
  int id = landmark->GetId();
  lane_landmarks_.insert({id, landmark});
}

LaneLandmark::Ptr MapGraph::GetLandmark(int id) {
  if (!lane_landmarks_.count(id)) {
    return nullptr;
  } else {
    return lane_landmarks_.at(id);
  }
}

std::map<int, KDTree::Ptr> MapGraph::GetLandmarkTrees(
    const std::set<int> &tracking_id_set) const {
  std::map<int, KDTree::Ptr> lm_trees;
  for (auto it = tracking_id_set.begin(); it != tracking_id_set.end(); ++it) {
    int id = *it;
    if (!lane_landmarks_.count(id)) {
      std::cout << "Error: Map Graph Should Contain this Landmark\n";
    } else {
      lm_trees.insert(
          std::make_pair(id, lane_landmarks_.at(id)->GetSearchTree()));
      // std::cout << "Search tree points--------------------------------------------\n";
      const auto lanepoints = lane_landmarks_.at(id)->GetLanePoints();
      // for (auto it = lanepoints.begin(); it != lanepoints.end(); ++it) {
      //   std::cout << it->position.transpose() << std::endl;
      // }
    }
  }

  return lm_trees;
}
}  // namespace mono_lane_mapping
