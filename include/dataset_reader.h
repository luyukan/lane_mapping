#pragma once
#include <memory>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include "type_define.h"


namespace fs = boost::filesystem;

namespace mono_lane_mapping {
class DatasetReader {
 public:
  typedef std::shared_ptr<DatasetReader> Ptr;
  DatasetReader(const std::string &config_file);
  void ParseDataset();
  bool PopOutSyncData(Odometry &sync_odometry, FrameObservation &sync_frame_observation);

 private:
  void load_lane_observations(const std::string &dir);
  void load_pose(const std::string &dir);
  std::string config_file_;
  std::map<int64_t, Odometry> pose_data_;
  std::map<int64_t, FrameObservation> frame_observations_;
  std::map<int64_t, Odometry>::iterator data_iterator_;
};
}  // namespace mono_lane_mapping
