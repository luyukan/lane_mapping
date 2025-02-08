//
// Created by yukan on 25-1-23.
//
#pragma once
#include <memory>
#include <iostream>
#include <Eigen/Eigen>
#include <pangolin/pangolin.h>

namespace mono_lane_mapping
{
  class VisualizationVariable {
   public:
    std::shared_ptr<VisualizationVariable> Ptr;
    static VisualizationVariable &GetInstance();
    VisualizationVariable();
   private:
  };
} // namespace mono_lane_mapping