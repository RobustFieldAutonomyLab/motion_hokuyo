#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeNode.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <vector>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

using namespace std;
octomap::OcTreeNode* oldNode;
octomap::OcTreeNode* newNode;
bool oldOccupied;
bool newOccupied;
octomap::AbstractOcTree* oldTree;
octomap::OcTree* oldOctree;
octomap::AbstractOcTree* newTree;
octomap::OcTree* newOctree;
int intialize = 0;
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

void condenceList();
{
    for(i = 1; i <= points; i++)
    {
       for(p = last+1; motionPoints[p] == 0; p+offset)

       int starter = i*offset+1;

       motionPoints[starter] = 1;
       motionPoints[starter+1] = motionPoints[p+1];
       motionPoints[starter+2] = motionPoints[p+2];
       motionPoints[starter+3] = motionPoints[p+3];
       motionPoints[starter+4] = motionPoints[p+4];
       motionPoints[starter+5] = motionPoints[p+5];

    }
}

void publisher()
{
    condenceList();
    std_msgs::Int32MultiArray array;
    array.data.clear();

}

//add a new point to the list
void writeSpot()
{
    for(p = 1; motionPoints[p] == 1; p+offset);
    g = p + 1;
    h = p + 2;
    j = p + 3;
 
    motionPoints[p] = 1;
    motionPoints[g] = x;
    motionPoints[h] = y;
    motionPoints[j] = z;
    motionPoints[p+4] = 1;
    motionPoints[p+5] = 0;

    points++;
}

//remove a point from the list
void deleteSpot()
{
    motionPoints[p] = 0;
    motionPoints[g] = 0;
    motionPoints[h] = 0;
    motionPoints[j] = 0;
    motionPoints[p+4] = 0;
    motionPoints[p+5] = 0;

    points--;
}

//Determine if point is on list
bool searchList()
{
//add for?
    for(p = 1; motionPoints[p] == 0 && p <= points*offset; p+offset);
    g = p + 1;
    h = p + 2;
    j = p + 3;

    if(motionPoints[g] == x && motionPoints[h] == y && motionPoints[j] == z)
    {
       return true;
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
    if(intialize == 0)
    {
       oldTree = msgToMap(msg);
       oldOctree = dynamic_cast<octomap::OcTree*>(oldTree);
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
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_test");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/octomap_full", 1, &test);
    motionPoints.resize(1000000);
    ros::spin();
    return 0;
}
