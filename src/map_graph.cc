//
// Created by yukan on 25-1-20.
//

#include "map_graph.h"

namespace mono_lane_mapping
{
  MapGraph::MapGraph() {}

  MapGraph &MapGraph::GetInstance()
  {
    static MapGraph instance;
    return instance;
  }
  void MapGraph::Init() {}

  const std::map<int, LaneLandmark::Ptr> &MapGraph::GetLandmarks()
  {
    return lane_landmarks_;
  }

  int MapGraph::GetLaneLandmarkNum()
  {
    return lane_landmarks_.size();
  }

  void MapGraph::AddLandmark(const LaneLandmark::Ptr &landmark)
  {
    int id = landmark->GetId();
    lane_landmarks_.insert({id, landmark});
  }

  LaneLandmark::Ptr MapGraph::GetLandmark(int id)
  {
    if (!lane_landmarks_.count(id))
    {
      return nullptr;
    }
    else
    {
      return lane_landmarks_.at(id);
    }
  }

  std::map<int, KDTree::Ptr> MapGraph::GetLandmarkTrees(const std::set<int> &id)
  {
  }
} // namespace mono_lane_mapping
