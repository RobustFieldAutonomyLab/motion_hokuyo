cmake_minimum_required(VERSION 2.8.3)
project(motion_hokuyo)

find_package(catkin REQUIRED COMPONENTS roscpp octomap_ros octomap_msgs)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_hokuyo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(octomap_test octomap_test.cpp)
target_link_libraries(octomap_test ${catkin_LIBRARIES})

add_executable(max_z max_z.cpp)
target_link_libraries(max_z ${catkin_LIBRARIES})
target_link_libraries(max_z ${PCL_LIBRARIES})

add_executable(hokuyo_ground_filter hokuyo_ground_filter.cpp)
target_link_libraries(hokuyo_ground_filter ${catkin_LIBRARIES})
target_link_libraries(hokuyo_ground_filter ${PCL_LIBRARIES})

add_executable(motion_detector motion_detector_octomap.cpp)
target_link_libraries(motion_detector ${catkin_LIBRARIES})
