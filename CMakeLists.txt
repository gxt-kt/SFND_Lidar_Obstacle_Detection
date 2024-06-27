cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# add_definitions(-std=c++11)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(lidar_obstacle)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# yaml-cpp
find_package(yaml-cpp REQUIRED)

set(SRC src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
include_directories(yaml_config)
list(APPEND SRC yaml_config/yaml_config.cpp)

# add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp ransac3d.cpp ransac3d.h src/kdtree3d.h src/cluster3d.h src/cluster3d.cpp)
add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} yaml-cpp)
