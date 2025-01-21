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
      const std::map<uint64_t, LaneLandmark::Ptr>& GetLandmarks();
      uint64_t GetLaneLandmarkNum();
    private:
      std::map<uint64_t, LaneLandmark::Ptr> lane_landmarks_;
  };
};  // namespace mono_lane_mapping
