cmake_minimum_required(VERSION 2.8.3)
project(hector_slam_launch)

find_package(catkin REQUIRED)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

install(DIRECTORY rviz_cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz_cfg/
)
