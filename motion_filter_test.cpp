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

using namespace std;

class cropBoxFilter{
	
	public:

	cropBoxFilter() {
		objects_filtered_pub = n_.advertise<sensor_msgs::PointCloud2> ("motionless_cloud", 1);

		//sub_range = n_.subscribe("/block_objects", 1, &cropBoxFilter::take_values, this);

		sub_cloud = n_.subscribe("/velodyne_points", 1, &cropBoxFilter::objectFilter, this);

		points_in = false;
	}

	void take_values(const visualization_msgs::MarkerArray &raw_markers){

		if(raw_markers.markers.size() == 0){
			return;
		}

		//vector<vector<float> > scale_list;
		//vector<vector<float> > centers;
		vector<visualization_msgs::Marker> import_markers;

		/*for(int i = 0; i < 3; i++){
			vector<float> scale;
			raw_markers.markers.
		}*/

		for(int i = 0; i < raw_markers.markers.size(); i++){ //Takes points from marker array message and creates min and max xyz point values
			visualization_msgs::Marker temp_marker = raw_markers.markers.at(i);
			vector<float> temp_points;
			//Order is min_x min_y min_z max_x max_y max_z
			temp_points.push_back(temp_marker.pose.position.x - temp_marker.scale.x/2);
			temp_points.push_back(temp_marker.pose.position.y - temp_marker.scale.y/2);
			temp_points.push_back(temp_marker.pose.position.z - temp_marker.scale.z/2);
			temp_points.push_back(temp_marker.pose.position.x + temp_marker.scale.x/2);
			temp_points.push_back(temp_marker.pose.position.y + temp_marker.scale.y/2);
			temp_points.push_back(temp_marker.pose.position.z + temp_marker.scale.z/2);
			temp_points.push_back(0);

			mm_points.push_back(temp_points);
		}

		for (int i = 0; i < mm_points[0].size(); i++){
			ROS_INFO_STREAM(mm_points[0][i]);
		}
		ROS_WARN_STREAM(mm_points.size());
		points_in = true;
	}

	void objectFilter(const sensor_msgs::PointCloud2ConstPtr& msg){  //Seperate callback function that filters velodyne points for the ranges of segmented objects
		
		//if (points_in == false || mm_points.empty()){
		//	return;
		//}

		//for(int i = 0; i < mm_points.size(); i++){
		//	mm_points[i][6]++;
		//	if (mm_points[i][6] > 10){
		//		mm_points.erase(mm_points.begin()+i);
		//		i--;
		//	}
		//}

		pcl::PointCloud<pcl::PointXYZ> cloud; //Input cloud initialization
		pcl::PointCloud<pcl::PointXYZ> cloud_f;

		pcl::fromROSMsg(*msg, cloud);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(&cloud);  //setInputCloud in pcl library requires a shared pointer to a point cloud as input, this creates a pointer to cloud
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f_ptr(&cloud_f); 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>); 
		//for(int i=0; i < ranges.size(); i++){  //For loop performing cropbox filter for each object in the ranges vector
			int i = 0;
			/*Eigen::Vector4f minPoint; 
		      minPoint[0]=mm_points[i][0]-0.5;  // define minimum point x 
		      minPoint[1]=mm_points[i][1]-0.5;  // define minimum point y 
		      minPoint[2]=mm_points[i][2]-0.5;  // define minimum point z 
		     Eigen::Vector4f maxPoint; 
		      maxPoint[0]=mm_points[i][3]+0.5;  // define max point x 
		      maxPoint[1]=mm_points[i][4]+0.5;  // define max point y 
		      maxPoint[2]=mm_points[i][5]+0.5;  // define max point z 
*/

		/*	Eigen::Vector4f minPoint; 
		      minPoint[0]=0;  // define minimum point x 
		      minPoint[1]=0;  // define minimum point y 
		      minPoint[2]=0;  // define minimum point z 
		     Eigen::Vector4f maxPoint; 
		      maxPoint[0]=5;  // define max point x 
		      maxPoint[1]=5;  // define max point y 
		      maxPoint[2]=5;  // define max point z 


			pcl::CropBox<pcl::PointXYZ> cropFilter; 
		    cropFilter.setInputCloud (cloud_ptr); 
		    cropFilter.setMin(minPoint); 
		    cropFilter.setMax(maxPoint); 
*/
		//   	cropFilter.filter (*cloudOut);
		//} 

		//sensor_msgs::PointCloud2 filtered;
		//pcl::toROSMsg(cloud_f, filtered);

		//objects_filtered_pub.publish(filtered);

		ROS_WARN_STREAM("End of filter");

		return;
	}

	private:
	ros::NodeHandle n_;
	ros::Publisher objects_filtered_pub;
	ros::Subscriber sub_range;
	ros::Subscriber sub_cloud;
	bool points_in;
	vector<vector<float> > mm_points;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_filter_test");
    ros::NodeHandle nh;
    cropBoxFilter filter1;
    //ros::topic::waitForMessage<dynamixel_msgs::JointState>("/tilt_controller/state", ros::Duration(100));
    ros::spin();
    return 0;
}