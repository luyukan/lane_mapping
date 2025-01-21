# mono_lane_mapping C

This is a rough implementation of Python Repo [MonoLaneMapping](https://github.com/HKUST-Aerial-Robotics/MonoLaneMapping).

Tested on Ubuntu 24.04 with **GTSAM 4.2a9**

**Authors:** [Yukan Lu](yukan.lu@gmail.com), [Xubin Fan](https://github.com/jackfine/)

## DEPENDENCY

### Eigen

- **https://github.com/eigenteam/eigen-git-mirror**  

### GTSAM

- **https://github.com/borglab/gtsam**  


### Pangolin

- **https://github.com/stevenlovegrove/Pangolin**  


### YAML-CPP

- **sudo apt-get install libyaml-cpp-dev**  
   

## Ceres

- **https://github.com/ceres-solver/ceres-solver**  





## Introduction
### Compile
   Clone the repository and make:
```
    cd lane_mapping
    mkdir build
    cd build
    cmake ..
    make -j4
```

### Description
   In order to run the code without ros support, I slightly modify the python code from origin repo: lane_ui.py, you can see what I do if you check the diff from python/lane_ui.py and the origin one. To Simplify the code, we only map one kind of the lanes but not all of them, but you can simply apply the schema to the others [see dataset_reader.cc for the details]

### Run Example
```
    cd build
    ./lane_mapping_demo ../config.yaml  
```
    modify the path in config.yaml as it should fit your own file system




