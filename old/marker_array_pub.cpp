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
int old_marker_id = 0;

vector<float> points;

void markerPublisher(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray markerArray;

  int marker_id = 0;

  for(int i=0; i<old_marker_id+10; i++){
    marker.ns = "motion_array";
    marker.id = i;
    marker.action = visualization_msgs::Marker::DELETE;
    markerArray.markers.push_back(marker);
  }
  ma_pub.publish(markerArray);

  markerArray.markers.clear();

	int t = 0;

	points.clear();
	for (std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
		points.push_back(*it);
		t++;
	}

	//markerArray.action = 3;
	for(int i=1; i<=msg->data.size(); i+=6) {
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "motion_array";
    	marker.id = marker_id;
    	marker.type = 2;
    	marker.action = visualization_msgs::Marker::ADD;

    	//double scale = octomap::OcTreeBaseImpl->getResolution();
    	double scale = 0.4;
    	marker.scale.x = scale;
    	marker.scale.y = scale;
    	marker.scale.z = scale;

    	marker.pose.position.x = points[i+1];
    	marker.pose.position.y = points[i+2];
    	marker.pose.position.z = points[i+3];

    	marker.color.r = 1.0f;
    	marker.color.g = 1.0f;
    	marker.color.b = 1.0f;
    	marker.color.a = 1.0;
    	marker.lifetime = ros::Duration();


		markerArray.markers.push_back(marker);
    marker_id++;    	
	}
  old_marker_id = marker_id;
	ma_pub.publish(markerArray);
}

int main(int argc, char **argv ) {
  	ros::init(argc, argv, "marker_array_pub");
  	ros::NodeHandle n;
  	ros::Subscriber array_sub = n.subscribe("/filtered_octo", 1, markerPublisher);
  	ma_pub = n.advertise<visualization_msgs::MarkerArray>("filtered_octo_marker", 1);
  	ros::spin();
}
