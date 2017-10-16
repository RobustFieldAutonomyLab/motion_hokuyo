#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float64.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

template <typename T>
    bool IsInBounds(const T& value, const T& low, const T& high) {
    return !(value < low) && (value < high);
}

class cropBoxFilter{
	
	public:

	cropBoxFilter() {
		objects_filtered_pub = n_.advertise<sensor_msgs::PointCloud2> ("motionless_cloud", 1);

		sub_range = n_.subscribe("/block_objects", 1, &cropBoxFilter::take_values, this);

		sub_cloud = n_.subscribe("/velodyne_points", 1, &cropBoxFilter::objectFilter, this);

		points_in = false;

		inf = numeric_limits<float>::infinity();

		other_pub = n_.advertise<sensor_msgs::PointCloud2> ("moving_objects", 1);

		point_count = 0;
	}

	void take_values(const visualization_msgs::MarkerArray &raw_markers){

		if(raw_markers.markers.empty()){
			return;
		}

		//max_points.clear();
		//min_points.clear();

		//vector<vector<float> > scale_list;
		//vector<vector<float> > centers;
		vector<visualization_msgs::Marker> import_markers;

		for(int i = 0; i < raw_markers.markers.size(); i++){ //Takes points from marker array message and creates min and max xyz point values
			visualization_msgs::Marker temp_marker = raw_markers.markers.at(i);
            Eigen::Vector4f temp_points_min;
            Eigen::Vector4f temp_points_max;
			//Order is min_x min_y min_z max_x max_y max_z
			temp_points_min[0] = (temp_marker.pose.position.x - temp_marker.scale.x/2);
			temp_points_min[1] = (temp_marker.pose.position.y - temp_marker.scale.y/2);
			temp_points_min[2] = (temp_marker.pose.position.z - temp_marker.scale.z/2);
			temp_points_max[0] = (temp_marker.pose.position.x + temp_marker.scale.x/2);
			temp_points_max[1] = (temp_marker.pose.position.y + temp_marker.scale.y/2);
			temp_points_max[2] = (temp_marker.pose.position.z + temp_marker.scale.z/2);
			//temp_points_max.push_back(0); //Decay timer value

			point_count++;

			min_points.push_back(temp_points_min);
			max_points.push_back(temp_points_max);
		}
		/*point_count = 0;
		decay.push_back(point_count);
		if(decay.size() > 1){
			min_points.erase(min_points.begin(), min_points.begin()+decay[0]);
			max_points.erase(max_points.begin(), max_points.begin()+decay[0]);
			decay.erase(decay.begin());
		}*/
	}

	void objectFilter(const sensor_msgs::PointCloud2ConstPtr& msg){

		if(min_points.empty()){
			return;
		}

		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ> aux;
		pcl::PointCloud<pcl::PointXYZ> other;


		//pcl::fromROSMsg(*msg, cloud);
		//aux.header = cloud.header;
		//other.header = cloud.header;

		pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr filteredIndices(new pcl::PointCloud<pcl::PointXYZ>);
		vector<int> filteredIndices;

		pcl::fromROSMsg(*msg, *inputCloud);

		for (int i = 0; i < min_points.size(); i++) {
			pcl::CropBox<pcl::PointXYZ> cropper(true);
			cropper.setMax(max_points[i]);
			cropper.setMin(min_points[i]);
			cropper.setInputCloud(inputCloud);
			//cropper.filter(*filteredCloud);
			cropper.filter(filteredIndices);

			boost::shared_ptr<vector<int> > indicesptr(new vector<int>(filteredIndices));
			pcl::ExtractIndices<pcl::PointXYZ> deleteMotionPoints;
			deleteMotionPoints.setInputCloud(inputCloud);
			deleteMotionPoints.setIndices(indicesptr);
			deleteMotionPoints.setNegative(true);
			deleteMotionPoints.filter(*inputCloud);
		}


    	sensor_msgs::PointCloud2 moving;
		pcl::toROSMsg(*inputCloud, moving);
		other_pub.publish(moving);

	}

	private:
	ros::NodeHandle n_;
	ros::Publisher objects_filtered_pub;
	ros::Publisher other_pub;
	ros::Subscriber sub_range;
	ros::Subscriber sub_cloud;
	bool points_in;
	vector<Eigen::Vector4f > max_points;
	vector<Eigen::Vector4f > min_points;
	float inf;
	vector<int> decay;
	int range_size_last;
	int point_count;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_filter_test");
    ros::NodeHandle nh;
    cropBoxFilter filter1;
    ros::spin();
    return 0;
}