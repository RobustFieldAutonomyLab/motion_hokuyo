
#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

using namespace std;

vector<float> masterList;
vector<float> object[1000];
int n = 0;
int point;
bool go;

void removeMaster()
{
    masterList.erase(masterList.begin()+point, masterList.begin()+point+5);
}

void removeObject()
{

}

void pickFirstPoint()
{
    n++;

    for(int i = 1; i < 7; i++)
    {
       object[n][1] = masterList[i];
    }

    object[n][0] = 1;

    point = 1;
    removeMaster();
}

bool findNeighbors()
{
    
}

void objectID(const std_msgs::Float32MultiArray &msg)
{
//make masterList == msg

    while(masterList[1] != 0)
    {
       pickFirstPoint();
       go = findNeighbors();

       if(go == false)
       {
           removeObject();
           return;
       }

       
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/filtered_octo", 1, &objectID);
    return 0;
}
