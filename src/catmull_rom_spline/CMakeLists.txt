# Set the minimum version of CMake
cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(CatmullRomSplineProject VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen3 library (assuming it's installed)
find_package(Eigen3 REQUIRED)

# Include Eigen's header files
include_directories(${EIGEN3_INCLUDE_DIR})

# Add the executable target for your main program
add_executable(test_spline test_list.cpp catmull_rom_spline.cpp catmull_rom_spline_list.cpp)

# Link the Eigen3 library (no need for linking since Eigen is header-only)
target_link_libraries(test_spline Eigen3::Eigen)

# Optionally, set the installation path (if needed)
# install(TARGETS test_spline DESTINATION bin)
