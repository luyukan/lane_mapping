#pragma once
#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

#include "type_define.h"

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

 private:
  void load_preprocess_parameters(const YAML::Node &node);
  void load_lane_mapping_parameters(const YAML::Node &node);
  PreProcessParameter preprocess_parameter_;
  LaneMappingParameter lane_mapping_parameter_;
  std::string config_file_;

};
} // namespace mono_lane_mapping
