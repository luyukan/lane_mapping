#include <iostream>
#include <thread>
#include <yaml-cpp/yaml.h>


#include "dataset_reader.h"
#include "lane_mapper.h"

using namespace mono_lane_mapping;

int main(int argc, char** argv) {
  std::string config_file = argv[1];

  DatasetReader::Ptr dataset_reader = std::make_shared<DatasetReader>(config_file);
  dataset_reader->ParseDataset();

  Odometry pose;
  FrameObservation frame_observation;

  LaneMapper& lane_mapper = LaneMapper::GetInstance();
  lane_mapper.Init(config_file);
  while(dataset_reader->PopOutSyncData(pose, frame_observation)) {
    std::cout << "Processing: " << pose.timestamp << std::endl;
    lane_mapper.InputSyncData(pose, frame_observation);
//    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }


  return 0;
}