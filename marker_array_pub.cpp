#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OcTreeBaseImpl.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>

using namespace std;

ros::Publisher ma_pub;

vector<float> points;

void markerPublisher(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray markerArray;

	points.clear();
	for (std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
		points.push_back(*it);
	}

	ROS_INFO_STREAM("Finished for loop one");

	for(int i = 1; i < msg->data.size(); i +=6) {
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "motion_array";
    	marker.id = i;
    	marker.type = 1;
    	marker.action = visualization_msgs::Marker::ADD;

    	//double scale = octomap::OcTreeBaseImpl->getResolution();
    	double scale = 0.1;
    	marker.scale.x = scale;
    	marker.scale.y = scale;
    	marker.scale.z = scale;

    	marker.pose.position.x = points[i+1];
    	marker.pose.position.y = points[i+2];
    	marker.pose.position.z = points[i+3];

    	marker.color.r = 0.0f;
    	marker.color.g = 1.0f;
    	marker.color.b = 0.0f;
    	marker.color.a = 1.0;
    	marker.lifetime = ros::Duration();

    	ROS_WARN_STREAM(msg->data.size());

		markerArray.markers.push_back(marker);    	
	}
	ROS_INFO_STREAM("Publish");
	ma_pub.publish(markerArray);
}

int main(int argc, char **argv ) {
  	ros::init(argc, argv, "marker_array_pub");
  	ros::NodeHandle n;
  	ros::Subscriber array_sub = n.subscribe("/filtered_octo", 1, markerPublisher);
  	ma_pub = n.advertise<visualization_msgs::MarkerArray>("filtered_octo_marker", 1);
  	ros::spin();
}
