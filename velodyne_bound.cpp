//
// Created by paul on 10/16/17.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float64.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/extract_indices.h>
#include <tf/tf.h>

using namespace std;

class velodyneCrop{
public:

    velodyneCrop(){
        sub_cloud = n_.subscribe("/velodyne_points", 1, &velodyneCrop::cropper, this);
        other_pub = n_.advertise<sensor_msgs::PointCloud2> ("/velodyne_bound", 1);
    }

    void cropper(const sensor_msgs::PointCloud2ConstPtr& msg){

        pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(*msg, *inputCloud);

        Eigen::Vector4f points_min;
        Eigen::Vector4f points_max;

        //pcl::getMinMax3D<pcl::PointXYZI>(*inputCloud, points_min, points_max);

        //Order is min_x min_y min_z max_x max_y max_z
        int range = 10;
        points_min[0] = -range;
        points_min[1] = -range;
        points_min[2] = -range;
        points_min[3] = 1;
        points_max[0] = range;
        points_max[1] = range;
        points_max[2] = range;
        points_max[3] = 1;

        pcl::CropBox<pcl::PointXYZI> cropper(true);
        cropper.setMax(points_max);
        cropper.setMin(points_min);
        cropper.setInputCloud(inputCloud);
        cropper.filter(*filteredCloud);

        tf::TransformListener::transformPointCloud();

        sensor_msgs::PointCloud2 crop;
        pcl::toROSMsg(*filteredCloud, crop);
        //crop.header = msg->header;
        other_pub.publish(crop);

    }
private:
    ros::NodeHandle n_;
    ros::Subscriber sub_cloud;
    ros::Publisher other_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velodyne_bound");
    ros::NodeHandle nh;
    velodyneCrop crop;
    ros::spin();
}