//
// Created by yukan on 25-1-23.
//
#include "visualization/visualization_variable.h"

namespace mono_lane_mapping {
VisualizationVariable &VisualizationVariable::GetInstance() {
  static VisualizationVariable instance;
  return instance;
}
VisualizationVariable::VisualizationVariable() {}
}  // namespace mono_lane_mapping