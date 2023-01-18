cmake_minimum_required(VERSION 2.8.3)
project(data_outlier_detection)

set(CMAKE_BUILD_TYPE "Release")
# NOTE(gogojjh): do we need to specifically set the c++ version?
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fPIC")

find_package(catkin REQUIRED 
  COMPONENTS 
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_filters)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS 
  DEPENDS 
)

add_executable(data_topic_check_fp src/data_topic_check_fp.cpp)
target_link_libraries(data_topic_check_fp ${catkin_LIBRARIES})
