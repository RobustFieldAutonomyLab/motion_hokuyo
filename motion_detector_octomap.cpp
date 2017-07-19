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
int g;
int h;
int j;
int offset = 6;
int points = 0;
vector<float> motionPoints;
bool present;
int p;
int last = -5;
ros::Publisher pub;

//remove a point from the list
void deleteSpot()
{
    motionPoints.erase(motionPoints.begin()+p-1, motionPoints.begin()+p+4);

    points--;
}

//using a 75% non-dynamic filter
void filterNoise()
{
    p=1;

    for(i = 1; i <= points; i++)
    {

       float ratio = motionPoints[p+5]/(motionPoints[p+4]+motionPoints[p+5]);
       
       if(ratio >= 0.75)
       {
          deleteSpot();
       }

       p=p+offset;
    }
}

//send out filtered list
void publisher()
{
    filterNoise();
    std_msgs::Float32MultiArray array;
    array.data.clear();
int count;
    for(i = 1; i <= points*offset; i++)
    {
count++;
       array.data.push_back(motionPoints[i]);
    }

ROS_INFO_STREAM(count);
    pub.publish(array);
}

//add a new point to the list
void writeSpot()
{
    motionPoints.push_back(1);
    motionPoints.push_back(x);
    motionPoints.push_back(y);
    motionPoints.push_back(z);
    motionPoints.push_back(1);
    motionPoints.push_back(0);

    points++;
}

//Determine if point is on list
bool searchList()
{
    for(i = 1; i <= points; i++)
    {
       p=1;

       if(motionPoints[p+1] == x && motionPoints[p+2] == y && motionPoints[p+3] == z)
       {
          return true;
       }
    }

    return false;
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
       motionPoints[p+5] = motionPoints[p+5] + 1;
    }
}

//Locate Possible Dynamic Points
void test(const octomap_msgs::Octomap &msg)
{
    if(initialize == 0)
    {
       oldTree = msgToMap(msg);
       oldOctree = dynamic_cast<octomap::OcTree*>(oldTree);
       initialize = 1;
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
       oldOctree = newOctree;
       publisher();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_test");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/octomap_full", 1, &test);
    pub=nh.advertise<std_msgs::Float32MultiArray>("filtered_octo", 1);
    ros::spin();
    return 0;
}
