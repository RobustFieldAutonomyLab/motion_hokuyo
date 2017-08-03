#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeNode.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <octomap/OcTreeBaseImpl.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
octomap::OcTreeNode* oldNode;
octomap::OcTreeNode* newNode;
bool oldOccupied;
bool newOccupied;
octomap::AbstractOcTree* oldTree;
octomap::OcTree* oldOctree;
octomap::AbstractOcTree* newTree;
octomap::OcTree* newOctree;
int initialize = 0;
float x;
float y;
float z;
int i;
int offset = 6;
int points = 0;
vector<float> motionPoints;
vector<float> holder;
bool present;
int p;
ros::Publisher pub_1;
ros::Publisher pub_2;
int runs = -1;
float ratioThresh = 0.75; //percent non-dynamic for deletion
float obsThresh = 0.66; //percent unobserved for deletion
int old_marker_id = 0;

//look into priority observation

//Determine if point is on list
bool searchList()
{
    p = 1;

    for(i = 1; i <= points; i++)
    {
       if((motionPoints[p+1] == x) && (motionPoints[p+2] == y) && (motionPoints[p+3] == z))
       {
          return true;
       }

       p = p + offset;
    }

    return false;
}

//Correct Deletion Errors
void correction()
{
    int s = 1;
    
    for(int l = 1; l <= points; l++)
    {
       x = holder[s+1];
       y = holder[s+2];
       z = holder[s+3];

       int k = 1;

       for(int t = 1; t <= points; t++)
       {
          if(motionPoints[k+1] == x && motionPoints[k+2] == y && motionPoints[k+3] == z)
          {
              motionPoints[k+5] = holder[s+5];
          }

          k = k + offset;
       }
 
       s = s + offset;
   }
}

//remove a point from the list
void deleteSpot()
{
    motionPoints.erase(motionPoints.begin()+p-1, motionPoints.begin()+p+5);
    points--;
}

//using a ratioThresh% non-dynamic filter
void filterNoise()
{
    p = 1;
    int hold = points;

    for(i = 1; i <= hold; i++)
    {
       float ratio = abs(motionPoints[p+5])/(motionPoints[p+4]+abs(motionPoints[p+5]));

       if(ratio >= ratioThresh)
       {
          holder = motionPoints;
          deleteSpot();
          correction();
       }

       if(ratio < ratioThresh)
       {
          p = p + offset;
       }
    }
}

//remove points not observed for obsThres% of runs
void filterUnobserved()
{
    int hold = points;
    p = 1;

    for(i = 1; i <= hold; i++)
    {
       int observed = runs - motionPoints[p] + 1;
       float seen = (abs(motionPoints[p+5]) + motionPoints[p+4])/observed;

       if(seen < obsThresh)
       {
          holder = motionPoints;
          deleteSpot();
          correction();
       }

       else
       {
          p = p + offset;
       }
     }    
}

void visualPublisher()
{
visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerArray;
    int marker_id = 0;
    int p = 7;

    for(int i = 0; i < old_marker_id; i++)
    {
        marker.ns = "motion_array";
        marker.id = i;
        marker.action = visualization_msgs::Marker::DELETE;
        markerArray.markers.push_back(marker);
    }

    pub_2.publish(markerArray);

        double r = (rand() % 10)/10.0;
        double g = (rand() % 10)/10.0;
        double b = (rand() % 10)/10.0;

        for(int temp = 1; temp <= points; temp++)
        {
             marker.header.frame_id = "/base_link";
             marker.header.stamp = ros::Time::now();
             marker.ns = "motion_array";

             marker.id = marker_id;
             marker.type = 1;

             marker.action = visualization_msgs::Marker::ADD;

             double scale = 0.3;
 
             marker.scale.x = scale;
             marker.scale.y = scale;
             marker.scale.z = scale;

             marker.pose.position.x = motionPoints[p+1];
    	     marker.pose.position.y = motionPoints[p+2];
    	     marker.pose.position.z = motionPoints[p+3];

    	     marker.color.r = (r);
    	     marker.color.g = (g);
    	     marker.color.b = (b);
    	     marker.color.a = 1.0; 
             marker.lifetime = ros::Duration();

             markerArray.markers.push_back(marker);
             marker_id++;

             p = p + offset;
        }

    old_marker_id = marker_id;

    pub_2.publish(markerArray);
}

//send out filtered list
void publisher()
{
    filterNoise();
    filterUnobserved();

    std_msgs::Float32MultiArray array;
    array.data.clear();
    int count;

    for(i = 0; i <= points*offset; i++)
    {
       count++;
       array.data.push_back(motionPoints[i]);
    }

    int holder = (count - 1) / offset;

    array.data.push_back(holder);

    for(i = 1; i <= points; i++)
    {
       p = p + offset;
    }

    ROS_INFO_STREAM(holder);

    pub_1.publish(array);
visualPublisher();
//    ros::Duration(1).sleep();
}

void writeSpot()
{
    motionPoints.push_back(runs);
    motionPoints.push_back(x);
    motionPoints.push_back(y);
    motionPoints.push_back(z);
    motionPoints.push_back(1);
    motionPoints.push_back(0);

    points++;
}



//Add/Update a Point to the List
void addMotionList()
{
    present = searchList();

    if(present == false)
    {
       writeSpot();
    }

    else
    {
       motionPoints[p+4] = motionPoints[p+4] + 1;
    }
}

//discredit a point on the list
void subtractMotionList()
{
    present = searchList();

    if(present == true)
    {
       motionPoints[p+5] = motionPoints[p+5] - 1;
    }
}

//Locate Possible Dynamic Points
void test(const octomap_msgs::Octomap &msg)
{
    runs++;

    if(initialize == 0)
    {
       oldTree = msgToMap(msg);
       oldOctree = dynamic_cast<octomap::OcTree*>(oldTree);
       initialize = 1;
       return;
    }

    else
    {
       newTree = msgToMap(msg);
       newOctree = dynamic_cast<octomap::OcTree*>(newTree);

       for(octomap::OcTree::leaf_iterator it = newOctree->begin_leafs(), end=newOctree->end_leafs(); it!=end; ++it)
       {
           x = it.getX();
           y = it.getY();
           z = it.getZ();

           if(oldOctree->search(x,y,z) != NULL)
           {
              oldNode = oldOctree->search(x,y,z);
              oldOccupied = oldOctree->isNodeOccupied(oldNode);

              newNode = newOctree->search(x,y,z);
              newOccupied = newOctree->isNodeOccupied(newNode);

              if (oldOccupied != newOccupied)
              {
                  addMotionList();
              }

              else
              {
                  subtractMotionList();
              }
           }
       }

       oldOctree->clear();
       oldOctree = newOctree;
       publisher();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_detector");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/octomap_full", 1, &test);
    pub_1=nh.advertise<std_msgs::Float32MultiArray>("filtered_octo", 1);
    pub_2=nh.advertise<visualization_msgs::MarkerArray>("motion_array", 1);
    motionPoints.push_back(0);
    ros::spin();
    return 0;
}
