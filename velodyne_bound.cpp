//
// Created by paul on 10/16/17.
//

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float64.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/pcl_base.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>