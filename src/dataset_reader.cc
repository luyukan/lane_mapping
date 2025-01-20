#include "dataset_reader.h"

namespace mono_lane_mapping {
DatasetReader::DatasetReader(const std::string &config_file)
    : config_file_(config_file) {}

void DatasetReader::ParseDataset() {
  try {
    YAML::Node yml = YAML::LoadFile(config_file_);
    std::string dataset_dir = yml["dataset_dir"].as<std::string>();

    std::string lanes_observation_dir = dataset_dir + "/lanes_predict";
    load_lane_observations(lanes_observation_dir);

    std::string odometry_dir = dataset_dir + "/pose";
    load_pose(odometry_dir);

    std::cout << "Pose Loaded: " << pose_data_.size() << std::endl;
    std::cout << "FrameObservation Loaded: " << frame_observations_.size()
              << std::endl;

    data_iterator_ = pose_data_.begin();

  } catch (const YAML::Exception &e) {
    std::cerr << "Error reading YAML: " << e.what() << std::endl;
  }
}

bool DatasetReader::PopOutSyncData(Odometry &sync_odometry,
                                   FrameObservation &sync_frame_observation) {
  if (data_iterator_ != pose_data_.end()) {
    auto it_frame_observation = frame_observations_.find(data_iterator_->first);
    if (it_frame_observation == frame_observations_.end()) {
      return false;
    }
    sync_odometry = data_iterator_->second;
    sync_frame_observation = it_frame_observation->second;
    return true;

  } else {
    return false;
  }
}

void DatasetReader::load_lane_observations(const std::string &dir) {
  fs::path path(dir);
  for (fs::directory_iterator dir_iter(path);
       dir_iter != fs::directory_iterator(); ++dir_iter) {
    if (fs::is_regular_file(dir_iter->status())) {
      std::string lane_predict_file = dir_iter->path().string();
      int n = lane_predict_file.find_last_of("/");
      int m = lane_predict_file.find_last_of(".");
      int64_t timestamp =
          std::atoll(lane_predict_file.substr(n + 1, m - n - 1).c_str());
      FrameObservation frame_observation;
      frame_observation.timestamp = timestamp;
      std::ifstream fin(lane_predict_file, std::ios::in);
      std::string line;
      int local_id = 0;
      while (getline(fin, line)) {
        std::vector<std::string> split_strings;
        boost::split(split_strings, line, boost::is_any_of(" "));
        LaneObservation lane_observation;
        lane_observation.local_id = local_id;
        ++local_id;
        for (size_t i = 0; i < split_strings.size() / 4; ++i) {
          LanePoint lane_point;
          lane_point.point_wcs.x() = std::atof(split_strings[4 * i].c_str());
          lane_point.point_wcs.y() =
              std::atof(split_strings[4 * i + 1].c_str());
          lane_point.point_wcs.z() =
              std::atof(split_strings[4 * i + 2].c_str());
          lane_point.visibility = std::atof(split_strings[4 * i + 3].c_str());
          lane_observation.lane_points.push_back(lane_point);
        }
        frame_observation.lane_observations.push_back(lane_observation);
      }
      fin.close();

      frame_observations_[timestamp] = frame_observation;
    }
  }
}

void DatasetReader::load_pose(const std::string &dir) {
  fs::path path(dir);
  for (fs::directory_iterator dir_iter(path);
       dir_iter != fs::directory_iterator(); ++dir_iter) {
    if (fs::is_regular_file(dir_iter->status())) {
      // std::cout << "File: " << dir_iter->path().string() << std::endl;
      std::string pose_file = dir_iter->path().string();
      int n = pose_file.find_last_of("/");
      int m = pose_file.find_last_of(".");
      int64_t timestamp =
          std::atoll(pose_file.substr(n + 1, m - n - 1).c_str());

      std::ifstream fin(pose_file, std::ios::in);
      std::string line;
      getline(fin, line);
      Eigen::Vector3d twb;
      Eigen::Quaterniond qwb;
      std::stringstream ss(line);
      ss >> twb.x() >> twb.y() >> twb.z() >> qwb.x() >> qwb.y() >> qwb.z() >>
          qwb.w();
      Odometry pose;
      pose.qwb = qwb;
      pose.twb = twb;
      pose.timestamp = timestamp;
      pose_data_[timestamp] = pose;
      fin.close();
    }
  }
}

}  // namespace mono_lane_mapping
