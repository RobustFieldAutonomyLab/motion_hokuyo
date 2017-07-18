cmake_minimum_required(VERSION 2.8.3)
project(motion_hokuyo)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	octomap_ros
	octomap_msgs
	rospy
	sensor_msgs
	cv_bridge
	dynamic_reconfigure
	message_generation
	image_transport
	tf
	pcl_ros
	pcl_conversions
	geometry_msgs
	nav_msgs
	)


find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

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

#add_executable(marker_array_pub marker_array_pub.cpp)
#target_link_libraries(marker_array_pub ${catkin_LIBRARIES})

add_executable(plane_extract_ransac plane_extract_ransac.cpp)
target_link_libraries(plane_extract_ransac ${catkin_LIBRARIES})
