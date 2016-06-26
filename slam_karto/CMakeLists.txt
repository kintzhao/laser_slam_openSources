cmake_minimum_required(VERSION 2.8.3)
project(slam_karto)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    rosconsole
    roscpp
    sensor_msgs
    tf
    visualization_msgs
)
find_package(Eigen REQUIRED)

SET( CMAKE_BUILD_TYPE Debug  )

MESSAGE("EIGEN_INCLUDE_DIRS = " ${EIGEN_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
 ${PROJECT_SOURCE_DIR}/include/open_karto
 ${PROJECT_SOURCE_DIR}/include/sparse_bundle_adjustment
)

catkin_package()

file(GLOB           SOURCES_
                             ${PROJECT_SOURCE_DIR}/include/open_karto/src/*.cpp
			     ${PROJECT_SOURCE_DIR}/include/sparse_bundle_adjustment/src/*.cpp
     )


# SBA library
add_library(sba   ${SOURCES_})
target_link_libraries(sba       cxsparse)
#target_link_libraries(sba   blas lapack cholmod cxsparse) 

add_executable(slam_karto src/slam_karto.cpp src/spa_solver.cpp     )
target_link_libraries(slam_karto ${catkin_LIBRARIES}  sba )


