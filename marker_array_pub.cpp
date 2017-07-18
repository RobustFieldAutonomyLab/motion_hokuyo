#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OcTreeBaseImpl.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>

ros::Publisher ma_pub;


void markerPublisher(const std_msgs::Int32MultiArray::ConstPtr& msg) {
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray markerArray;

	int array[100000];
	int t = 0;
	for (std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
		array[t] = *it;
		t++;
	}


	for(int i=0; i<=(sizeof(msg)/sizeof(float)); i=i*6) {
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "motion_array";
    	marker.id = i;
    	marker.type = 1;
    	marker.action = visualization_msgs::Marker::ADD;

    	//double scale = octomap::OcTreeBaseImpl->getResolution();
    	double scale = 0.3;
    	marker.scale.x = scale;
    	marker.scale.y = scale;
    	marker.scale.z = scale;

    	marker.pose.position.x = array[i+1];
    	marker.pose.position.y = array[i+2];
    	marker.pose.position.z = array[i+3];

    	marker.color.r = 0.0f;
    	marker.color.g = 1.0f;
    	marker.color.b = 0.0f;
    	marker.color.a = 1.0;
    	marker.lifetime = ros::Duration();

		markerArray.markers.push_back(marker);    	
	}
	ma_pub.publish(markerArray);
}

int main( int argc, char** argv ) {
  	ros::init(argc, argv, "marker_array_pub");
  	ros::NodeHandle n;
  	ros::Subscriber array_sub = n.subscribe("/filtered_octo", 1, markerPublisher);
  	ma_pub = n.advertise<visualization_msgs::MarkerArray>("filtered_octo_marker", 1);
  	ros::spin();
}