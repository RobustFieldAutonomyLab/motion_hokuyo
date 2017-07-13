#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeNode.h>
#include <octomap/AbstractOccupancyOcTree.h>

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
float motionPoints[100][4];
int spots = 100;

void motionList()
{
    for(i = 1; motionPoints[i][1] != 0  && i != spots+1; i++)
    if(i > spots)
    {
       spots = spots + 10;
       motionPoints.resize(spots);
    }


}
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
                  motionList();
              }
           }
       }
    }

//size_t l;
//l = octree->calcNumNodes();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_test");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/octomap_full", 1, &test);
    ros::spin();
    return 0;
}
