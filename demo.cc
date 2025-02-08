#include <iostream>
#include <thread>

#include "dataset_reader.h"
#include "lane_mapper.h"
#include "system_param.h"
#ifdef VIEWER_ON
#include "visualization/pangolin_viewer.h"

#endif

using namespace mono_lane_mapping;

int main(int argc, char** argv) {
  std::string config_file = argv[1];

  SystemParam& system_param = SystemParam::GetInstance();

  DatasetReader::Ptr dataset_reader =
      std::make_shared<DatasetReader>(config_file);
  dataset_reader->ParseDataset();

  system_param.Init(config_file);
  system_param.LoadParameters();

#ifdef VIEWER_ON
  auto& pangolin_viewer = PangolinViewer::GetInstance();
  pangolin_viewer.Init();
  pangolin_viewer.Start();
#endif

  Odometry pose;
  FrameObservation frame_observation;

  LaneMapper& lane_mapper = LaneMapper::GetInstance();
  lane_mapper.Init();
  while (dataset_reader->PopOutSyncData(pose, frame_observation)) {
    std::cout << "Processing: " << pose.timestamp << std::endl;
    lane_mapper.InputSyncData(pose, frame_observation);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }


  while(true) {

  }
  
  return 0;
}