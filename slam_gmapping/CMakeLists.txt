cmake_minimum_required(VERSION 2.8)
project(gmapping)

find_package(catkin REQUIRED nav_msgs roscpp  tf   rosbag_storage)
find_package(Boost REQUIRED signals)

include_directories(${Boost_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS}  ${PROJECT_SOURCE_DIR}/include  ${PROJECT_SOURCE_DIR}/include/gmapping  ${PROJECT_SOURCE_DIR}/include/openslam_gmapping)
include_directories( ${PROJECT_SOURCE_DIR}/src)

catkin_package()

add_subdirectory(${PROJECT_SOURCE_DIR}/include/openslam_gmapping)


add_executable(slam_gmapping_demo src/slam_gmapping.cpp src/main.cpp)
target_link_libraries(slam_gmapping_demo ${Boost_LIBRARIES} ${catkin_LIBRARIES} gridfastslam scanmatcher sensor_base sensor_range sensor_odometry utils)

add_executable(slam_gmapping_replay_demo src/slam_gmapping.cpp src/replay.cpp)
target_link_libraries(slam_gmapping_replay_demo ${Boost_LIBRARIES} ${catkin_LIBRARIES} gridfastslam scanmatcher sensor_base sensor_range sensor_odometry utils)


if(catkin_EXPORTED_TARGETS)
  add_dependencies(slam_gmapping_demo ${catkin_EXPORTED_TARGETS})
endif()

if(catkin_EXPORTED_TARGETS)
add_dependencies(slam_gmapping_replay_demo ${catkin_EXPORTED_TARGETS})
endif()

