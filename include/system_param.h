#pragma once
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>

#include "type_define.h"

#define VIEWER_ON

namespace mono_lane_mapping {
class SystemParam {
 public:
  typedef std::shared_ptr<SystemParam> Ptr;
  SystemParam();
  static SystemParam &GetInstance();
  void Init(const std::string &config_file);
  void LoadParameters();
  const PreProcessParameter &GetPreProcessParameters();
  const LaneMappingParameter &GetLaneMappingParameters();
  const LaneAssoParameter &GetLaneAssoParameters();

 private:
  void load_preprocess_parameters(const YAML::Node &node);
  void load_lane_mapping_parameters(const YAML::Node &node);
  void load_lane_asso_parameters(const YAML::Node &node);
  PreProcessParameter preprocess_parameter_;
  LaneMappingParameter lane_mapping_parameter_;
  LaneAssoParameter lane_asso_parameter_;
  std::string config_file_;
};
}  // namespace mono_lane_mapping
