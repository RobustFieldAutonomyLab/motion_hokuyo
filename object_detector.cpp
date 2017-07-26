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
float threshold = 0.3;
int obtain = 0;
float total;
int offset = 6;
int qualifier = 10;
int initialize = 0;
ros::Publisher pub_1;

//adjusts min and max values for x, y, or z depending on input
void redo(int k, int jump, int miner, int maxer)
{
//k => object #
//jump => set for x, y, or z (x=1; y=2; z=3)
//miner => minimum for letter (x=1; y=3; z=5)
//maxer => minimum for letter (x=2; y=4; z=6)

    int p = 7;
    float max = object[k][p+jump];
    float min = object[k][p+jump];

    for(p = 13; p <= object[k][0] * offset + 7; p+=offset)
    {
        if(min > object[k][p+jump])
        {
           min = object[k][p+jump];
        }

        if(max < object[k][p+jump])
        {
           max = object[k][p+jump];
        }
    }

    object[k][miner] = min;
    object[k][maxer] = max;
}

//Take a point off the master lisst
void removeMaster(int point)
{
//point => starting value of point data on list

    masterList.erase(masterList.begin()+point, masterList.begin()+point+6);
    total--;
}

//take an entire object off the object list
void removeObject(int k)
{
//k => object number

    object.erase(object.begin()+k);
    n--;
}

//take a point off an object and access if any min/max was impacted
void removeFromObject(int point, int k)
{
//point => starting value of point data on list
//k => object number

    int x = object[k][point+1];
    int y = object[k][point+2];
    int z = object[k][point+3];

    object[k].erase(object[k].begin()+point-1, object[k].begin()+point+5);
    object[k][0]--;

    if(object[k][0] == 0)
    {
       removeObject(k);
    }

    else
    {
       if(x == object[k][1] || x == object[k][2])
       {
          redo(k, 1, 1, 2);
       }

       if(y == object[k][3] || y == object[k][4])
       {
          redo(k, 2, 3, 4);
       }

       if(z == object[k][5] || z == object[k][6])
       {
          redo(k, 3, 5, 6);
       }
    }
}

//add new point to object, adjust max/min if needed, remove point from master list
void addToObject(int p, int k)
{
//p => starting position of point on master list to be added
//k => object number

    for(int i = p; i < p+6; i++)
    {
       object[k].push_back(masterList[i]);
    } 

    object[k][0]++;

    if(object[k][1] > masterList[p+1])
    {
       object[k][1] = masterList[p+1];
    }

    if(object[k][2] < masterList[p+1])
    {
       object[k][2] = masterList[p+1];
    }

    if(object[k][3] > masterList[p+2])
    {
       object[k][3] = masterList[p+2];
    }

    if(object[k][4] < masterList[p+2])
    {
       object[k][4] = masterList[p+2];
    }

    if(object[k][5] > masterList[p+3])
    {
       object[k][5] = masterList[p+3];
    }

    if(object[k][6] < masterList[p+3])
    {
       object[k][6] = masterList[p+3];
    }

    removeMaster(p);
}

//use first point on mater list as start for new object, sets max/min, remove from master list
void pickFirstPoint()
{
    n++;
    object.push_back(vector<float>());//add new object
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

//see if any points on master list neighbor a point in the object
//true = found neighbor(s); false = no neighbors
bool findNeighbors(int home, int k)
{
//home => starting position of point data in object to be tested
//k => object number examined

    int good = 0;
    int hold = total;
    int p = 1;

    for(int i = 1; i <= hold; i++)
    {
        float test = sqrt(pow((object[k][home+1] - masterList[p+1]), 2) +
                          pow((object[k][home+2] - masterList[p+2]), 2) +
                          pow((object[k][home+3] - masterList[p+3]), 2));

        if(test <= threshold)
        {
           addToObject(p, k);

           good = 1;
        }

        else
        {
           p = p + offset;
        }
    }

    if(good == 0)
    {
       return false;
    }

    else
    {
       return true;
    }
}

//determine if points on new master list belong to any object
void examineObjects()
{
    for(int k = 1; k <= n; k++)
    {
        int hold = total;
        int p = 1;

        for(int j = 2; j <= object[k][0]; j++)
        {
            findNeighbors(j * offset + 1, k);
        }
    }
}

//remove points from objects that are not on new master list
//remove points from master list that are already in an object
void deadPoints()
{
    int objects = n;
    int ob = 1;

    for(int k = 1; k <= objects; k++)
    {
        int remember = n;
        int space = object[ob][0];
        int spot = 7;

        for(int j = 1; j <= space; j++)
        {
           bool match = false;
           int p = 1;
           int hold = total;

           for(int i = 1; i <= hold; i++)
           {
               if(masterList[p+1] == object[ob][spot+1] &&
                  masterList[p+2] == object[ob][spot+2] &&
                  masterList[p+3] == object[ob][spot+3])
               {
                   removeMaster(p);
                   match = true;
               }

               else
               {
                   p = p + offset;
               }
           }

           if(match == false)
           {
               removeFromObject(spot, ob);
           }

           else
           {
               spot = spot + offset;
           }
        }

        if(n == remember)
        {
           ob++;
        }
    }
}

//publish object points to array for marker array visualization
void visualPublisher()
{
    std_msgs::Float32MultiArray array;
    array.data.clear();

    array.data.push_back(0);

    for(int k = 1; k <= n; k++)
    {
        for(int j = 7; j <= object[k][0] * offset; j++)
        {
           array.data.push_back(object[k][j]);
        }
    }

    pub_1.publish(array);
}

//take in new master list, convert to vector, remove duplicates, identicals, and those belonging to other objects and then create new objects, then publish
void objectID(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    masterList.clear();
    for (std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
    {
        masterList.push_back(*it);
    }

    total = masterList.back();

    if(n!=0)
    {
       deadPoints();
       examineObjects();
    }

    while(total != 0)
    {
       pickFirstPoint();
       go = findNeighbors(7, n);

       if(go == false)
       {
           removeObject(n);
       }

       else
       {
          for(int i = 2; i <= object[n][0]; i++)
          {
              findNeighbors(i * offset + 1, n);
          }
if(object[n][0] >= 10)
{
ROS_ERROR_STREAM(object[n][0]);
}
          if(object[n][0] <= qualifier)
          {
              removeObject(n);
          }
       }
    }

   // ROS_WARN_STREAM(n);
    visualPublisher();
}

//main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    ros::Subscriber sub_1=nh.subscribe("/filtered_octo", 1, &objectID);
    object.push_back(vector<float>()); //occupies object[0]
    pub_1=nh.advertise<std_msgs::Float32MultiArray>("objects", 1);
    ros::spin();
    return 0;
}
