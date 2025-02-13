#pragma once 
#include <memory>
#include <iostream>
#include <Eigen/Eigen>

#include "lane_landmark.h"

namespace mono_lane_mapping {
  class MapGraph {
    public:
      typedef std::shared_ptr<MapGraph> Ptr;
      MapGraph();
      static MapGraph &GetInstance();
      void Init();
      const std::map<int, LaneLandmark::Ptr>& GetLandmarks();
      int GetLaneLandmarkNum();
      void AddLandmark(const LaneLandmark::Ptr &landmark);
      LaneLandmark::Ptr GetLandmark(int id);
      std::map<int, KDTree::Ptr> GetLandmarkTrees(const std::set<int> &id);
    private:
      std::map<int, LaneLandmark::Ptr> lane_landmarks_;
  };
};  // namespace mono_lane_mapping
