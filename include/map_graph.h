#pragma once 

#include <Eigen/Eigen>

namespace mono_lane_mapping {
  class MapGraph {
    public:
      typdef std::shared_ptr<MapGraph> Ptr;
      MapGraph();
      static MapGraph &GetInstance();
      void Init();
    private:
  }
};  // namespace mono_lane_mapping
