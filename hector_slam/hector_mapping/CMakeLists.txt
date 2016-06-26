cmake_minimum_required(VERSION 2.8.3)
project(hector_mapping)


find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp nav_msgs visualization_msgs tf message_filters laser_geometry tf_conversions message_generation)


find_package(Boost REQUIRED COMPONENTS thread signals)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})


add_message_files(
  FILES
  HectorDebugInfo.msg
  HectorIterData.msg
)


generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS roscpp nav_msgs visualization_msgs tf message_filters laser_geometry tf_conversions message_runtime
  DEPENDS Eigen
)


include_directories(include include/hector_slam_lib)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hector_mapping
  src/HectorDebugInfoProvider.h
  src/HectorDrawings.h
  src/HectorMappingRos.h
  src/HectorMappingRos.cpp
  src/main.cpp
  src/PoseInfoContainer.cpp
  src/PoseInfoContainer.h
)

add_dependencies(hector_mapping hector_mapping_generate_messages_cpp)


target_link_libraries(hector_mapping
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


install(TARGETS hector_mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

