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
vector<vector<float> > tempPoints;
vector<vector<float> > importList;
ros::Publisher ma_pub;
float boundingBox = 5;
float bound = 0.4;
int old_marker_id = 0;
int min_points_in_bound = 12;

template <typename T>
    bool IsInBounds(const T& value, const T& low, const T& high) {
    return !(value < low) && (value < high);
}

void pointBounder() {  //DEPRECATED Main code transferred to nearest neighbor function

	//vector< vector<float> > tempPoints;
	for(int j = 0; j < importList.size(); j++){  	//Set a bounding box using global variable bound on all points in importList with reference to the first point in importList vector
													//Send all points inside bounding box, including the first reference point, to tempPoints
		if ((IsInBounds(importList[0][0], importList[j][0]-boundingBox, importList[j][0]+boundingBox) && (IsInBounds(importList[0][1], importList[j][1]-boundingBox, importList[j][1]+boundingBox) && (IsInBounds(importList[0][2], importList[j][2]-boundingBox, importList[j][2]+boundingBox))))){
			tempPoints.push_back(importList[j]);
		}
	}

	/*int i = 0;
	for(int i; i < tempPoints.size(); i++){
		string  s = to_string(tempPoints[i][3]);// + "," + to_string(tempPoints[i][1]) + "," + to_string(tempPoints[i][2]);
		ROS_INFO_STREAM(s);
	}
	return;*/ //tempPoints;
}

void nearestNeighbor(){ //Should be split to allow for more modularity

	vector< vector<float> > tempObject; //initialize object to be added to objects vector
	if(tempPoints.size() >= min_points_in_bound){  //Only begin adding to object if there are a certain number of points in bounds
		tempObject.push_back(tempPoints[0]);  //Add first point of tempPoints to tempObject
		tempPoints.erase (tempPoints.begin());
		int i = 0;
		while(ros::ok()){ //Object creation while loop
			float min_dist = (2*bound); //minimum distance initialized as bounding box size
			int min_location;

			for (int j = 0; j < tempPoints.size(); j++){ //Loop through all points in the tempPoints vector
				float distance = sqrt(pow((tempObject[i][0]-tempPoints[j][0]),2)+pow((tempObject[i][1]-tempPoints[j][1]),2)+pow((tempObject[i][2]-tempPoints[j][2]),2)); //Calculate euclidean distance between all points inside of cube segment
				//tempPoints[j][3] = distance; //REMOVE? Add distance value to slot 3 of points vector
				if (distance < min_dist){
					//min_dist = distance; //REMOVE? deprecated?
					min_location = j; //set min location to be the current index value of the vector if distance value is smallest
					min_dist = distance;
				}
			}
			if (min_dist == 2*bound){ //if min distance is still initialized value then break because no nearest neighbors within threshold
				//tempObject.clear();

				break;
			}
			else{
				tempObject.push_back(tempPoints[min_location]); //Add lowest distance point to tempObject
				tempPoints.erase(tempPoints.begin()+min_location); //Erase this point from tempPoints
				for(int k = 0; k < importList.size(); k++){
					if(importList[k][3] == tempPoints[min_location][3]){
						importList.erase(importList.begin()+k);
					}
				}
			//ros::Duration(0.01).sleep();
			}
			++i; //Increment the tempObject point being used for nearest neighbor calculations, as we want to use the most recent tempObject point
		}
	}
	else{ //If no points are detected dont add the single point as an object and erase the point
		importList.erase(importList.begin());
	}
	/*for(int i = 0; i < tempObject.size(); i++){ //Output all tempObject points to console 
			string  s = to_string(tempObject[i][0]) + "," + to_string(tempObject[i][1]) + "," + to_string(tempObject[i][2]);
			//ROS_WARN_STREAM(s);
		}
	*/
	objects.push_back(tempObject); //Add temporary object (vector of points) to objects vector (vector of objects)
}

void markerPublisher() {
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray markerArray;
	int marker_id = 0;

	//ROS_INFO_STREAM(objects.size());

	/*if(old_marker_id < msg->data.size()){
    	old_marker_id = msg->data.size();
  	}*/

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
			marker.header.frame_id = "/map";
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
  
	//objects.clear();
	tempPoints.clear();
	importList.clear();
	objects.clear();
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	bool first = 1; //Value that keeps track of the first run of the for loop
	int points_index = 0; //Value that indexes each of the points to keep consistency between importPoints and tempPoints
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
		node.push_back(points_index); //Slot in vector for point index value
		++points_index;
		importList.push_back(node);
	}
	//ROS_WARN_STREAM("Entering nearestNeighbor func");
	//while((tempPoints.size() != 0)&&(ros::ok())){
	while(importList.size() > 1 && ros::ok()){
		pointBounder();
		nearestNeighbor();
		tempPoints.clear();
	}

	markerPublisher();
	return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/filtered_octo", 1, &arrayCallback);
    ma_pub = nh.advertise<visualization_msgs::MarkerArray>("object_marker", 1);
    srand (time(0));
    ros::spin();
}