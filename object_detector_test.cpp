#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

vector<vector<vector<float> > > objects;
vector<vector<float> > tempPoints;
vector<vector<float> > importList;
ros::Publisher ma_pub;
float bound = 1;

template <typename T>
    bool IsInBounds(const T& value, const T& low, const T& high) {
    return !(value < low) && (value < high);
}

void pointBounder() {  //DEPRECATED Main code transferred to nearest neighbor function

	//vector< vector<float> > tempPoints;
	for(int j = 0; j < importList.size(); j++){  //Add all points inside the bounding box to tempPoints
		if ((IsInBounds(importList[0][0], importList[j][0]-bound, importList[j][0]+bound) && (IsInBounds(importList[0][1], importList[j][1]-bound, importList[j][1]+bound) && (IsInBounds(importList[0][2], importList[j][2]-bound, importList[j][2]+bound))))){
			tempPoints.push_back(importList[j]);
		}
	}

	/*int i = 0;
	for(int i; i < tempPoints.size(); i++){
		string  s = to_string(tempPoints[i][0]) + "," + to_string(tempPoints[i][1]) + "," + to_string(tempPoints[i][2]);
		ROS_INFO_STREAM(s);
	}*/
	return; //tempPoints;
}

void nearestNeighbor(int start_point){ //Should be split to allow for more modularity

	vector< vector<float> > tempObject; //initialize object to be added to objects vector
	if(tempPoints.size() > 1){
		tempObject.push_back(tempPoints[0]);  //Add first point of tempPoints to tempObject
		tempPoints.erase (tempPoints.begin());
		int i = 0;
		ROS_ERROR_STREAM(tempPoints.size());
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
					//ROS_ERROR_STREAM(min_dist);
				}
			}
			if (min_dist == 2*bound){ //if min distance is still initialized value then break because no nearest neighbors within threshold
				//tempObject.clear();
				break;
			}
			else{
				ROS_INFO_STREAM(tempPoints.size());
				tempObject.push_back(tempPoints[min_location]); //Add lowest distance point to tempObject
				tempPoints.erase(tempPoints.begin()+min_location); //Erase this point from tempPoints
			}
			++i;
			//ROS_WARN_STREAM(tempObject.size());
			//ROS_ERROR_STREAM(tempPoints.size());
		}
	}
	else{ //If no points are detected dont add the single point as an object and erase the point
		//tempObject.push_back(tempPoints[0]);
		tempPoints.erase(tempPoints.begin());
		//ROS_ERROR_STREAM("else pushback");
	}
	//ROS_WARN_STREAM(tempObject.size());
	for(int i = 0; i < tempObject.size(); i++){ //Output all tempObject points to console 
			string  s = to_string(tempObject[i][0]) + "," + to_string(tempObject[i][1]) + "," + to_string(tempObject[i][2]);
			//ROS_WARN_STREAM(s);
		}
	objects.push_back(tempObject); //Add temporary object (vector of points) to objects vector (vector of objects)
}

void markerPublisher() {
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray markerArray;

	//ROS_INFO_STREAM(objects.size());

	//markerArray.action = 3;
	for(int i=0; i<objects.size(); i++){
		//ROS_WARN_STREAM(objects[i].size());
		for(int j=0; j<objects[i].size(); j++) {



			marker.header.frame_id = "/map";
			marker.header.stamp = ros::Time::now();
			marker.ns = "motion_array";
    		marker.id = j;
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

    		marker.color.r = 0.0f;
    		marker.color.g = 0.0f;
    		marker.color.b = 1.0f;
    		marker.color.a = 1.0;
    		marker.lifetime = ros::Duration(0);

    		//ROS_WARN_STREAM(msg->data.size());

			markerArray.markers.push_back(marker);    	
		}
	}
	//ROS_INFO_STREAM("Publish");
	ma_pub.publish(markerArray);

	//objects.clear();
	tempPoints.clear();
	importList.clear();
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	int first = 1;
	int start_point = 0; //Sets index point 0 as the first point to check in the vector
	for (std::vector<float>::const_iterator it = msg->data.begin(); it < msg->data.end(); it+=3){
		//std::vector<float>::const_iterator it = msg->data.begin();
		if(first == 1){
			++it;	
			first = 0;
		}
		vector<float> node;
		for (int i = 1; i < 4; ++i){
			++it;
			node.push_back(*it);
			//ROS_ERROR_STREAM(*it);
		}
		//node.push_back(0); //REMOVE? Adds deprecated extra slot in vector for distance measurement
		importList.push_back(node);
	}
	//ROS_WARN_STREAM("Entering nearestNeighbor func");
	//while((tempPoints.size() != 0)&&(ros::ok())){
		pointBounder();
		nearestNeighbor(start_point);
	//}

	markerPublisher();
	return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/filtered_octo", 1, &arrayCallback);
    ma_pub = nh.advertise<visualization_msgs::MarkerArray>("object_marker", 1);
    ros::spin();
}