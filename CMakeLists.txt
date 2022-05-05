cmake_minimum_required(VERSION 3.0.0)
project(lidar_curb_detection)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  eigen_conversions
)
set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/boost.cmake)
include(cmake/openmp.cmake)
include(cmake/opencv.cmake)
include(cmake/limbo.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy
  std_msgs
  pcl_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  eigen_conversions
)


file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(curb_detection_node src/curb_detection_node.cpp ${ALL_SRCS})
target_link_libraries(curb_detection_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})