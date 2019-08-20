//ros basic header file
#include "ros/ros.h"
#include "math.h"
//message header file
#include "geometry_msgs/Vector3.h"
#include "macaron/base_frame.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

//global
int node_index;
bool flag=1;
darknet_ros_msgs::BoundingBoxes traffic_signs;
std_msgs::Int32MultiArray available_path;
std_msgs::Float64 speed_reduction;

int crosswalk_start = 88;
int crosswalk_end   = 120;
int static_obstacle_start  = 191;
int static_obstacle_end    = 226;
int safe_zone_start        = 140;
int safe_zone_end          = 174;
float traffic_flag[9]={0,0,0,0,0,0,0,0,0};

void baseCALLBACK(const std_msgs::Int32::ConstPtr& index)
{
    node_index = index->data;
}
void trafficCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& detection)
{
for(int i=0; i<9; i++) traffic_flag[i]=0;

for(int i=0 ; i < detection->bounding_boxes.size(); i++){
    traffic_flag[detection->bounding_boxes[i].Class]=detection->bounding_boxes[i].probability;
    }

}

void mission_identify()
{
    speed_reduction.data = 1.0*flag;
    available_path.data.resize(3);
    available_path.data[0] = 0; //차선 변경 불가
    available_path.data[1] = 1;
    available_path.data[2] = 0;
    if(node_index >= crosswalk_start && node_index <= crosswalk_end)                 //cross walk
    {
    //    printf("Mission : Cross walk\n");
    }
    else if(node_index >= static_obstacle_start && node_index <= static_obstacle_end) //static obstacle
    {
        available_path.data[0] = 1; //좌측으로 차선변경 가능
        available_path.data[1] = 1;
        available_path.data[2] = 0;
        printf("Mission : Static obsatacle\n");
    }
    else if(node_index >= safe_zone_start && node_index <= safe_zone_end)             //safe zone
    {
        speed_reduction.data = 0.7*flag;
        printf("Mission : Safe zone\n");
    }
    else                                                                              // normal course
        printf("Mission : Normal course\n");
    printf("---------------------------------------------------\n");

    if(traffic_flag[0]>0) flag=1;
    else if(traffic_flag[1]>0) flag=0;
}


int main(int argc, char **argv)
{
    //Initialize the node and resister it to the ROSMASTER
    ros::init(argc, argv, "mission_identifiying");
    //Declare node handle
    ros::NodeHandle nh;
    //Declare sub,pub
    ros::Subscriber mission_location_sub  = nh.subscribe("index",1,baseCALLBACK);
    ros::Subscriber traffic_sign_identify = nh.subscribe("/darknet_ros/bounding_boxes",100,trafficCallBack);
    ros::Publisher available_path_pub     = nh.advertise<std_msgs::Int32MultiArray>("/available_path",10);
    ros::Publisher speed_reduction_pub    = nh.advertise<std_msgs::Float64>("/speed_reduction",10);
    
    while(ros::ok())
    {
        ros::spinOnce(); 
        mission_identify();
        speed_reduction_pub.publish(speed_reduction);
        available_path_pub.publish(available_path);

    }   
}
