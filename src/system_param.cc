#include "system_param.h"

namespace mono_lane_mapping {
SystemParam::SystemParam() {}

SystemParam &SystemParam::GetInstance() {
  static SystemParam instance;
  return instance;
}

void SystemParam::Init(const std::string &config_file) {
  config_file_ = config_file;
}

void SystemParam::LoadParameters() {
  try {
    YAML::Node yml = YAML::LoadFile(config_file_);

    YAML::Node preprocess_node = yml["preprocess"];
    load_preprocess_parameters(preprocess_node);

    YAML::Node lane_mapping_node = yml["lane_mapping"];
    load_lane_mapping_parameters(lane_mapping_node);

  } catch (const YAML::Exception &e) {
    std::cerr << "Error reading YAML: " << e.what() << std::endl;
  }
}

void SystemParam::load_preprocess_parameters(const YAML::Node &node) {
  preprocess_parameter_.downsample_distance = node["downsample_distance"].as<double>();

  std::cout << "PreProcess Parameters Loaded\n";
  std::cout << "preprocess_parameter.downsample_distance: " << preprocess_parameter_.downsample_distance << std::endl;
  std::cout << "####################################################################\n";

}
void SystemParam::load_lane_mapping_parameters(const YAML::Node &node) {
  lane_mapping_parameter_.ctrl_points_chord = node["ctrl_points_chord"].as<double>();
  lane_mapping_parameter_.candidate_angle_thresh = node["candidate_angle_thresh"].as<double>();

  std::cout << "Lane Mapping Parameters Loaded\n";
  std::cout << "lane_mapping_parameter.ctrl_points_chord: " << lane_mapping_parameter_.ctrl_points_chord << std::endl;
  std::cout << "lane_mapping_parameter.candidate_angle_thresh: " << lane_mapping_parameter_.candidate_angle_thresh
            << std::endl;
  std::cout << "####################################################################\n";
}

const PreProcessParameter& SystemParam::GetPreProcessParameters() {
  return preprocess_parameter_;
}
const LaneMappingParameter& SystemParam::GetLaneMappingParameters() {
  return lane_mapping_parameter_;
}

} // namespace mono_lane_mapping
