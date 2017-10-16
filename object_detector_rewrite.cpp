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

using namespace std;
typedef pcl::PointXYZI  PointType;

template <typename T>
    bool IsInBounds(const T& value, const T& low, const T& high) {
    return !(value < low) && (value < high);
}

class objectSegment{

public:
	objectSegment() {
		inf = numeric_limits<float>::infinity();

		boundingBox = 0.4;

		min_object_size = 4;

		old_marker_id = 0;

		old_block_id = 0;

		ranges_in = false;

		sub_array=n_.subscribe("/filtered_octo", 1, &objectSegment::objectSegmentation, this);

		//sub_cloud=n_.subscribe("/velodyne_points", 1, &objectSegment::objectFilter, this);

		ma_pub = n_.advertise<visualization_msgs::MarkerArray>("object_marker_test", 1);

		block_ma_pub = n_.advertise<visualization_msgs::MarkerArray>("block_objects", 1);

		objects_filtered_pub = n_.advertise<sensor_msgs::PointCloud2>("objects_filtered", 1);
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

	void markerPublisher(vector<vector<vector<float> > > &objects) {
		visualization_msgs::Marker marker;
		visualization_msgs::MarkerArray markerArray;
		int marker_id = 0;

	  	for(int i=0; i<old_marker_id+10; i++){
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
	}

	void blockMarkerPub(vector<vector<float> > ranges){
		visualization_msgs::Marker marker;
		visualization_msgs::MarkerArray markerArray;
		int marker_id = 0;

	  	for(int i=0; i<old_block_id+10; i++){
			marker.ns = "block_object_array";
			marker.id = i;
			marker.action = visualization_msgs::Marker::DELETE;
			markerArray.markers.push_back(marker);
	  	}
	    block_ma_pub.publish(markerArray);

		markerArray.markers.clear();

		//markerArray.action = 3;
		for(int i=0; i<ranges.size(); i++){
			//ROS_WARN_STREAM(objects[i].size());
			double r = (rand() % 10)/10.0;
			double g = (rand() % 10)/10.0;
			double b = (rand() % 10)/10.0;
			
			marker.header.frame_id = "/map";
			marker.header.stamp = ros::Time::now();
			marker.ns = "block_object_array";
	    	marker.id = marker_id;
	    	marker.type = 1;
	    	//marker.action = visualization_msgs::Marker::DELETEALL;
	   		marker.action = visualization_msgs::Marker::ADD;

	    	//double scale = octomap::OcTreeBaseImpl->getResolution();
	    	//double scale = 0.3;
	    	marker.scale.x = (ranges[i][1]-ranges[i][0])+0.8;
	    	marker.scale.y = (ranges[i][3]-ranges[i][2])+0.8;
	    	marker.scale.z = (ranges[i][5]-ranges[i][4])+0.8;

	    	marker.pose.position.x = (ranges[i][0]+ranges[i][1])/2;
	    	marker.pose.position.y = (ranges[i][2]+ranges[i][3])/2;
	    	marker.pose.position.z = (ranges[i][4]+ranges[i][5])/2;

	    	marker.color.r = (r);
	    	marker.color.g = (g);
	    	marker.color.b = (b);
	    	marker.color.a = 1.0;
	    	marker.lifetime = ros::Duration();

	    	//ROS_WARN_STREAM(msg->data.size());

			markerArray.markers.push_back(marker);
			marker_id++;
		}
		old_block_id = marker_id;

		//ROS_ERROR_STREAM(objects.size());//markerArray.markers.size());

		block_ma_pub.publish(markerArray);
	}

	void objectSegmentation(const std_msgs::Float32MultiArray::ConstPtr& msg){

		ROS_WARN_STREAM("Begin of segment func");

		vector<vector<vector<float> > > objects;
		vector<vector<float> > unboundPoints = importPoints(msg);

		while(!unboundPoints.empty()){
			vector<vector<float> > boundPoints;
			vector<float> min_max(6); //Index of points is in min_x,max_x,min_y,max_y,min_z,max_z

			if (boundPoints.empty()){
				boundPoints.push_back(unboundPoints[0]);
				for(int j = 0; j <= 2; j++){ //Determine max and min of one object
					min_max[2*j+1] = boundPoints[0][j];
					min_max[2*j] = boundPoints[0][j];
				}
				min_max.push_back(0); //Added value to use as decay timer later in decay_ranges
				unboundPoints.erase(unboundPoints.begin());
			}
			//ROS_ERROR_STREAM("New object");
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

				/*for(int i = 0; i < min_max.size(); i++){
					string  s = to_string(min_max[i]);// + "," + to_string(boundPoints[i][1]) + "," + to_string(boundPoints[i][2]);
					ROS_INFO_STREAM(s);
				}*/

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
				ranges.push_back(min_max);
			}
			boundPoints.clear();
			min_max.clear();
		}

		markerPublisher(objects);
		blockMarkerPub(ranges);
		ranges_in = true;

		for(int i=0; i < ranges.size(); i++){
			decay_ranges.push_back(ranges[i]);
		} 
		ROS_INFO_STREAM(ranges.size());
		ROS_INFO_STREAM(decay_ranges.size());
		ranges.clear();
		return;
	}



private:
	ros::NodeHandle n_;
	float boundingBox;
	int min_object_size;
	int old_marker_id;
	int old_block_id;
	ros::Publisher ma_pub;
	ros::Publisher block_ma_pub;
	ros::Publisher objects_filtered_pub;
	bool ranges_in;
	ros::Subscriber sub_array;
	ros::Subscriber sub_cloud;
	vector<vector<float> > ranges;
	vector<vector<float> > decay_ranges;
	float inf;
};


/*void objectFilter(const sensor_msgs::PointCloud2ConstPtr& msg){
	
	pcl::PointCloud<PointType> cloud;
	pcl::fromROSMsg(*msg, cloud);

	Eigen::Vector4f minPoint; 
      minPoint[0]=0;  // define minimum point x 
      minPoint[1]=0;  // define minimum point y 
      minPoint[2]=0;  // define minimum point z 
     Eigen::Vector4f maxPoint; 
      minPoint[0]=5;  // define max point x 
      minPoint[1]=6;  // define max point y 
      minPoint[2]=7;  // define max point z 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn(new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>); 

    pcl::CropBox<PointCloud2> cropFilter; 
    cropFilter.setInputCloud (cloud); 
    cropFilter.setMin(minPoint); 
    cropFilter.setMax(maxPoint); 

   	cropFilter.filter (*cloudOut); 
}*/




int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    srand (time(0));
   	objectSegment segment;
    ros::spin();
}