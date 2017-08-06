#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h> 

using namespace std;

vector<vector<vector<float> > > objects;
float boundingBox = 0.4;
int min_object_size = 7;
int old_marker_id = 0;
ros::Publisher ma_pub;

template <typename T>
    bool IsInBounds(const T& value, const T& low, const T& high) {
    return !(value < low) && (value < high);
}

vector<vector<float> > importPoints(const std_msgs::Float32MultiArray::ConstPtr& msg){
	vector<vector<float> > importList;
	bool first = 1; //Value that keeps track of the first run of the for loop
	//int points_index = 0; //Value that indexes each of the points to keep consistency between importPoints and tempPoints
	for (std::vector<float>::const_iterator it = msg->data.begin(); it < msg->data.end(); it+=3){
		if(first == 1){ //Check for first loop
			++it;	
			first = 0;
		}
		vector<float> node;
		for (int i = 1; i < 4; ++i){
			++it;
			node.push_back(*it);
			//ROS_ERROR_STREAM(*it);
		}
		node.push_back(0);//points_index); //Slot in vector for point index value
		//++points_index;
		importList.push_back(node);
	}
	return importList;
}

vector<vector<vector<float> > > objectSegment (vector<vector<float> > unboundPoints){	//Set a bounding box using global variable bound on all points in importList with reference to the first point in importList vector
	vector<vector<vector<float> > > objects;
	while(!unboundPoints.empty()){
		vector<vector<float> > boundPoints;
		vector<float> min_max(6); //Index of points is in min_x,max_x,min_y,max_y,min_z,max_z

		if (boundPoints.empty()){
			boundPoints.push_back(unboundPoints[0]);
			for(int j = 0; j <= 2; j++){ //Determine max and min of one object
				min_max[2*j+1] = boundPoints[0][j];
				min_max[2*j] = boundPoints[0][j];
			}
			unboundPoints.erase(unboundPoints.begin());
		}
		ROS_ERROR_STREAM("New object");
		int old_size = 0;
		while(old_size != boundPoints.size()){  //Loops for as long as new points are being added to bound points
			for(int j = 0; j <= 2; j++){ //Determine max and min of one object
				for(int i = 0; i < boundPoints.size(); i++){
					if (min_max[2*j+1] < boundPoints[i][j]){ //Max values
						min_max[2*j+1] = boundPoints[i][j];
					}
					else if (min_max[2*j] > boundPoints[i][j]){ //Min values
						min_max[2*j] = boundPoints[i][j];
					}
				}
			}

			for(int i = 0; i < min_max.size(); i++){
				string  s = to_string(min_max[i]);// + "," + to_string(boundPoints[i][1]) + "," + to_string(boundPoints[i][2]);
				ROS_INFO_STREAM(s);
			}
			ROS_ERROR_STREAM("loop");
			//ros::Duration(0.2).sleep();

			old_size = boundPoints.size();
			for(int j = 0; j < unboundPoints.size(); j++){  //Send all points inside bounding box, including the first reference point, to boundPoints								
				//		X value bound 																			Y value bound 																	Z value bound
				if ((IsInBounds(unboundPoints[j][0], min_max[0]-boundingBox, min_max[1]+boundingBox) && (IsInBounds(unboundPoints[j][1], min_max[2]-boundingBox, min_max[3]+boundingBox) && (IsInBounds(unboundPoints[j][2], min_max[4]-boundingBox, min_max[5]+boundingBox))))){
					boundPoints.push_back(unboundPoints[j]);
					unboundPoints.erase(unboundPoints.begin()+j);
					j--;
				}
			}
		}

		/*for(int i = 0; i < boundPoints.size(); i++){
			string  s = to_string(boundPoints[i][0]) + "," + to_string(boundPoints[i][1]) + "," + to_string(boundPoints[i][2]);
			ROS_INFO_STREAM(s);
		}*/

		if(boundPoints.size() > min_object_size){
			objects.push_back(boundPoints);
		}
		boundPoints.clear();
		min_max.clear();
	}
	ROS_WARN_STREAM(objects.size());
	return objects;
}

void markerPublisher(vector<vector<vector<float> > > objects) {
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray markerArray;
	int marker_id = 0;

  	for(int i=0; i<old_marker_id; i++){
		marker.ns = "object_array";
		marker.id = i;
		marker.action = visualization_msgs::Marker::DELETE;
		markerArray.markers.push_back(marker);
  	}
  	ma_pub.publish(markerArray);

	markerArray.markers.clear();

	//markerArray.action = 3;
	for(int i=0; i<objects.size(); i++){
		//ROS_WARN_STREAM(objects[i].size());
		double r = (rand() % 10)/10.0;
		double g = (rand() % 10)/10.0;
		double b = (rand() % 10)/10.0;
		
		for(int j=0; j<objects[i].size(); j++) {
			marker.header.frame_id = "/map";//"/map";
			marker.header.stamp = ros::Time::now();
			marker.ns = "object_array";
    		marker.id = marker_id;
    		marker.type = 1;
    		//marker.action = visualization_msgs::Marker::DELETEALL;
   			marker.action = visualization_msgs::Marker::ADD;

    		//double scale = octomap::OcTreeBaseImpl->getResolution();
    		double scale = 0.3;
    		marker.scale.x = scale;
    		marker.scale.y = scale;
    		marker.scale.z = scale;

    		marker.pose.position.x = objects[i][j][0];
    		marker.pose.position.y = objects[i][j][1];
    		marker.pose.position.z = objects[i][j][2];

    		marker.color.r = (r);
    		marker.color.g = (g);
    		marker.color.b = (b);
    		marker.color.a = 1.0;
    		marker.lifetime = ros::Duration();

    		//ROS_WARN_STREAM(msg->data.size());

			markerArray.markers.push_back(marker);
			marker_id++; 	
		}
	}
	old_marker_id = marker_id;

	//ROS_ERROR_STREAM(objects.size());//markerArray.markers.size());

	ma_pub.publish(markerArray);
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	markerPublisher(objectSegment(importPoints(msg)));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/filtered_octo", 1, &arrayCallback);
    ma_pub = nh.advertise<visualization_msgs::MarkerArray>("object_marker_test", 1);
    srand (time(0));
    ros::spin();
}