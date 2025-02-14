//
// Created by yukan on 25-1-23.
//
#pragma once
#include <memory>
#include <mutex>
#include <iostream>
#include <Eigen/Eigen>

namespace mono_lane_mapping
{
  class VisualizationVariable {
   public:
    std::shared_ptr<VisualizationVariable> Ptr;
    static VisualizationVariable &GetInstance();
    VisualizationVariable();
    std::mutex& GetVisualizationMutex();
   private:
    std::mutex variable_mutex_;
  };
} // namespace mono_lane_mapping