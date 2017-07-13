#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeNode.h>
#include <octomap/AbstractOccupancyOcTree.h>

using namespace std;
octomap::OcTreeNode* p;
octomap::OcTreeNode y;
bool x;

void test(const octomap_msgs::Octomap &msg)
{
octomap::point3d min; min.x() = -1; min.y() = -1; min.z() = -1;
octomap::point3d max; max.x() = 1; max.y() = 1; max.z() = 1;

    octomap::AbstractOcTree* tree = msgToMap(msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    /*ROS_WARN_STREAM(octree->search(0, 0, 0));
    p = octree->search(0,0,0);
    x = octree->isNodeOccupied(p);
    ROS_ERROR_STREAM(x);*/

    for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min,max), end=octree->end_leafs_bbx(); it!= end; ++it)
    {
         p = octree->search(it.getX(), it.getY(), it.getZ());
         x = octree->isNodeOccupied(p);
ROS_WARN_STREAM(x);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_test");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/octomap_full", 1, &test);
    ros::spin();
    return 0;
}
