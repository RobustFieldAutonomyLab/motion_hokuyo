#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64.h>

using namespace std;

vector<float> masterList;
vector<vector<float>> object;
int n = 0;
bool go;
float threshold = 0.03;
int obtain = 0;
int total;
int offset = 6;
int qualifier = 10;

void removeMaster(int point)
{
    masterList.erase(masterList.begin()+point-1, masterList.begin()+point+5);
    total--;
}

void removeObject()
{
    object.erase(object.begin()+n);
}

void pickFirstPoint()
{
    n++;
    object.push_back(vector<float>());
    object[n].push_back(1); //object[n][0]
    object[n].push_back(masterList[1]); //object[n][1] min X
    object[n].push_back(masterList[1]); //object[n][2] max x
    object[n].push_back(masterList[2]); //object[n][3] min y
    object[n].push_back(masterList[2]); //object[n][4] max y
    object[n].push_back(masterList[3]); //object[n][5] min z
    object[n].push_back(masterList[3]); //object[n][6] max z

    for(int i = 1; i < 7; i++)
    {
       object[n].push_back(masterList[i]);
    }

    removeMaster(1);
}

bool findNeighbors(int home)
{
    int good = 0;
    int hold = total;

    for(int p = 1; p <= hold*offset; p+offset)
    {
        float test = sqrt(pow((object[n][home+1] - masterList[p+1]), 2) +
                          pow((object[n][home+2] - masterList[p+2]), 2) +
                          pow((object[n][home+3] - masterList[p+3]), 2));

        if(test <= threshold)
        {
           for(int i = p; i < p+7; i++)
           {
              object[n].push_back(masterList[i]);
              object[n][0]++;
           }

           removeMaster(p);
           good = 1;
        }
    }

    if(good == 0)
    {
       return true;
    }

    else
    {
       return false;
    }
}

void obtainData(const std_msgs::Float64 &msg)
{
ROS_WARN_STREAM("total");
    total = msg.data;
    obtain = 1;
}

void objectID(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
ROS_INFO_STREAM("vector");
    masterList.clear();
    for (std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    {
        masterList.push_back(*it);
    }

    while(obtain == 0)
    {
       ros::spinOnce();
    }

    obtain = 0;

    while(total != 0)
    {
       pickFirstPoint();
       go = findNeighbors(7);

       if(go == false)
       {
           removeObject();
           return;
       }

       for(int i = 2; i <= object[n][0]; i++)
       {
           findNeighbors(i * offset + 1);
       }

       if(object[n][0] <= qualifier)
       {
           removeObject();
       }
    }

    ROS_WARN_STREAM(n);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    ros::Subscriber sub_1=nh.subscribe("/filtered_octo", 1, &objectID);
    ros::Subscriber sub_2=nh.subscribe("/motion_points", 1, &obtainData);
    object.push_back(vector<float>()); //occupies object[0]
    ros::spin();
    return 0;
}
