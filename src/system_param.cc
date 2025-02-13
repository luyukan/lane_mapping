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

    YAML::Node lane_asso_node = yml["lane_asso"];
    load_lane_asso_parameters(lane_asso_node);

  } catch (const YAML::Exception &e) {
    std::cerr << "Error reading YAML: " << e.what() << std::endl;
  }
}

void SystemParam::load_preprocess_parameters(const YAML::Node &node) {
  preprocess_parameter_.downsample_distance =
      node["downsample_distance"].as<double>();
  preprocess_parameter_.observation_pts_num_min =
      node["observation_pts_num_min"].as<int>();
  std::cout << "PreProcess Parameters Loaded\n";
  std::cout << "preprocess_parameter.downsample_distance: "
            << preprocess_parameter_.downsample_distance << std::endl;
  std::cout << "preprocess_parameter.observation_pts_num_min: "
            << preprocess_parameter_.observation_pts_num_min << std::endl;
  std::cout << "###############################################################"
               "#####\n";
}

void SystemParam::load_lane_asso_parameters(const YAML::Node &node) {
  lane_asso_parameter_.yaw_std = node["yaw_std"].as<double>();
  lane_asso_parameter_.translation_std = node["trans_std"].as<double>();
  lane_asso_parameter_.lane_width = node["lane_width"].as<double>();

  std::cout << "Lane Asso Parameters Loaded\n";
  std::cout << "lane_asso_parameter.yaw_std: " << lane_asso_parameter_.yaw_std
            << std::endl;
  std::cout << "lane_asso_parameter.translation_std: "
            << lane_asso_parameter_.translation_std << std::endl;
  std::cout << "lane_asso_parameter.lane_width: "
            << lane_asso_parameter_.lane_width << std::endl;
  std::cout << "###############################################################"
               "#####\n";
}
void SystemParam::load_lane_mapping_parameters(const YAML::Node &node) {
  lane_mapping_parameter_.ctrl_points_chord =
      node["ctrl_points_chord"].as<double>();
  lane_mapping_parameter_.candidate_angle_thresh =
      node["candidate_angle_thresh"].as<double>();
  lane_mapping_parameter_.sliding_window_size =
      node["sliding_window_size"].as<int>();
  std::vector<double> ctrl_noise_vector =
      node["ctrl_noise"].as<std::vector<double>>();
  for (size_t i = 0; i < ctrl_noise_vector.size(); ++i) {
    lane_mapping_parameter_.ctrl_noise[i] = ctrl_noise_vector.at(i);
  }
  lane_mapping_parameter_.keyframe_translation_thresh =
      node["keyframe_translation_thresh"].as<double>();

  std::cout << "Lane Mapping Parameters Loaded\n";
  std::cout << "lane_mapping_parameter.ctrl_points_chord: "
            << lane_mapping_parameter_.ctrl_points_chord << std::endl;
  std::cout << "lane_mapping_parameter.candidate_angle_thresh: "
            << lane_mapping_parameter_.candidate_angle_thresh << std::endl;
  std::cout << "lane_mapping_parameter.sliding_window_size: "
            << lane_mapping_parameter_.sliding_window_size << std::endl;
  std::cout << "lane_mapping_parameter.ctrl_noise: "
            << lane_mapping_parameter_.ctrl_noise.transpose() << std::endl;
  std::cout << "lane_mapping_parameter.keyframe_translation_thresh: "
            << lane_mapping_parameter_.keyframe_translation_thresh << std::endl;
  std::cout << "###############################################################"
               "#####\n";
}

const PreProcessParameter &SystemParam::GetPreProcessParameters() {
  return preprocess_parameter_;
}
const LaneMappingParameter &SystemParam::GetLaneMappingParameters() {
  return lane_mapping_parameter_;
}

const LaneAssoParameter &SystemParam::GetLaneAssoParameters() {
  return lane_asso_parameter_;
}

}  // namespace mono_lane_mapping
