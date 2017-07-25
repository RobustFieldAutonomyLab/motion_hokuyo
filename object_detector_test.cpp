#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

using namespace std;

vector<vector<vector<float> > > objects;
float bound = 5;

template <typename T>
    bool IsInBounds(const T& value, const T& low, const T& high) {
    return !(value < low) && (value < high);
}

/*void pointBounder() {  //DEPRECATED Main code transferred to nearest neighbor function

	for(int j = 0; j < importList.size(); j++){
		if ((IsInBounds(importList[j][0], low, high) && (IsInBounds(importList[j][1], low, high) && (IsInBounds(importList[j][2], low, high))))){
			tempPoints.push_back(importList[j]);
		}
	}
	int i = 0;
	for(int i; i < tempPoints.size(); i++){
		string  s = to_string(tempPoints[i][0]) + "," + to_string(tempPoints[i][1]) + "," + to_string(tempPoints[i][2]);
		ROS_INFO_STREAM(s);
	}
}*/

void nearestNeighbor(vector< vector<float> > importList, int start_point){ //Should be split to allow for more modularity
	vector< vector<float> > tempPoints;
	for(int j = 0; j < importList.size(); j++){  //Add all points inside the bounding box to tempPoints
		if ((IsInBounds(importList[j][0], -bound, bound) && (IsInBounds(importList[j][1], -bound, bound) && (IsInBounds(importList[j][2], -bound, bound))))){
			tempPoints.push_back(importList[j]);
		}
	}
	if (tempPoints.empty()){ //If no other point are inside the bound, mark point as noise
		
	}

	vector< vector<float> > tempObject; //initialize object to be added to 3d objects vector
	tempObject.push_back(tempPoints[0]);  //Add first point of tempPoints to tempObject
	tempPoints.erase (tempPoints.end()-1);
	while(ros::ok()){ //Object creation while loop
		float min_dist = (2*bound); //minimum distance initialized as bounding box size
		int min_location;

		for (int j = 0; j < tempPoints.size(); j++){ //Loop through all points in the tempPoints vector
			float distance = sqrt(pow((tempObject[0][0]-tempPoints[j][0]),2)+pow((tempObject[0][1]-tempPoints[j][1]),2)+pow((tempObject[0][2]-tempPoints[j][2]),2)); //Calculate euclidean distance between all points inside of cube segment
			tempPoints[j][3] = distance; //REMOVE? Add distance value to slot 3 of points vector
			if (distance < min_dist){
				min_dist = distance; //REMOVE? deprecated?
				min_location = j; //set min location to be the current index value of the vector if distance value is smallest
			}
		}
		if (min_dist == 2*bound){ //if min distance is still initialized value then break because no nearest neighbors within threshold
			tempObject.clear();
			break;
		}
		tempObject.insert(tempObject.begin(),tempPoints[min_location]); //Add lowest distance point to tempObject
		tempPoints.erase(tempPoints.begin()+min_location); //Erase this point from tempPoints

	}
	ROS_WARN_STREAM(tempObject.size());
	for(int i = 1; i < tempObject.size(); i++){ //Output all tempObject points to console 
			string  s = to_string(tempObject[i][0]) + "," + to_string(tempObject[i][1]) + "," + to_string(tempObject[i][2]);
			ROS_WARN_STREAM(s);
			ros::Duration(1).sleep();
			ROS_INFO_STREAM("Meme");
		}
	ROS_ERROR_STREAM("After publishing for");
	objects.push_back(tempObject); //Add temporary object (vector of points) to objects vector (vector of objects)
}


void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	vector< vector<float> > importList;
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
		node.push_back(0); //REMOVE? Adds deprecated extra slot in vector for distance measurement
		importList.push_back(node);
	}
	//ROS_WARN_STREAM("Entering nearestNeighbor func");
	nearestNeighbor(importList, start_point);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/filtered_octo", 1, &arrayCallback);
    ros::spin();
}