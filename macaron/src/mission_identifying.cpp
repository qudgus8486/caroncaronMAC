//ros basic header file
#include "ros/ros.h"
#include "math.h"
//message header file
#include "geometry_msgs/Vector3.h"
#include "macaron/base_frame.h"
#include "macaron/Floats_for_mission.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

//global
int node_index;
bool flag=1;
float traffic_flag[9]={0,0,0,0,0,0,0,0,0};

darknet_ros_msgs::BoundingBoxes traffic_signs;
std_msgs::Int32MultiArray available_path;
std_msgs::Float64 speed_reduction;


void baseCALLBACK(const std_msgs::Int32::ConstPtr& index)
{
    node_index = index->data;
}


void trafficCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& detection)
{
    for(int i=0; i<9; i++)
    {
        traffic_flag[i] = 0;
    }
    for(int i = 0 ; i < detection->bounding_boxes.size(); i++)
    {
        traffic_flag[detection->bounding_boxes[i].Class] = detection->bounding_boxes[i].probability;
    }

}


void laneCallBack(const macaron::Floats_for_mission::ConstPtr& mission)
{
    float deviation=mission->deviation;
    float left_curv=mission->left_curv;
    float right_curv=mission->right_curv;
    int line_type=mission->line_type;
    int error=mission->error;
    int fair_state=mission->fair_state;
    bool left_trun=mission->left_trun;
    bool right_trun=mission->right_trun;
}


void mission_identify()
{
    //기본 초기값
    speed_reduction.data = 1.0;
    available_path.data.resize(3);
    available_path.data[0] = 0; //차선 변경 불가
    available_path.data[1] = 1;
    available_path.data[2] = 0;

    //팔정도 모의 실험
    if(node_index >= 88 && node_index <= 120)      //cross walk
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n");  
    }
    else if(node_index >= 140 && node_index <= 174) //safe zone
    {
        speed_reduction.data = 0.7;
        printf("Mission : Safe zone\n");
    }
    else if(node_index >= 191 && node_index <= 226) //static obstacle
    {
        available_path.data[0] = 1; //좌측으로 차선변경 가능
        available_path.data[1] = 1;
        available_path.data[2] = 0;
        printf("Mission : Static obstacle\n");
    }
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");

    //만해광장 장애물 회피 실험
    /*
    available_path.data[0] = 0; //오른쪽으로 차선변경
    available_path.data[1] = 1;
    available_path.data[2] = 1;
    printf("Mission : Static obstacle\n");
    printf("\n---------------------------------------------------\n\n");
    */

    //k-city 예선
    /*
    if(node_index >= 124 && node_index <= 153)      //정적 장애물(드럼 통) 등장 구간
    {
        available_path.data[0] = 1; //좌측으로 차선변경 가능
        available_path.data[1] = 1;
        available_path.data[2] = 0; 
        printf("Mission : Static obstacle\n");
    }
    else if(node_index >= 216 && node_index <= 232) //신호등, 직진, 232가 정지선
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n"); 
    }
    else if(node_index >= 271 && node_index <= 285) //신호등, 좌회전, 285가 정지선
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n"); 
    }
    else if(node_index >= 316 && node_index <= 336) //돌방 장애물 등장 구간
    {
        printf("Mission : Dynamic obstacle\n");
    }
    else if(node_index >= 361 && node_index <= 380) //신호등, 좌회전, 380이 정지선
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n"); 
    }
    else if(node_index >= 411 && node_index <= 427) //신호등, 직진, 427이 정지선
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n"); 
    }
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
    */

    //k-city 본선(아직 주차 제외)
    /*
    if(node_index >= 213 && node_index <= 229)      //신호등, 좌회전, 229가 정지선
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n"); 
    }
    else if(node_index >= 267 && node_index <= 283) //신호등, 직진, 283이 정지선
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n"); 
    }
    else if(node_index >= 311 && node_index <= 400) //정적 장애물(대형장애물) 등장 구간
    {
        available_path.data[0] = 0; //우측으로 차선변경 가능
        available_path.data[1] = 1;
        available_path.data[2] = 1; 
        printf("Mission : Static obstacle\n");
    }
    else if(node_index >= 403 && node_index <= 419) //신호등, 직진, 419가 정지선
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n"); 
    }
    else if(node_index >= 885 && node_index <= 901) //신호등, 직진, 901이 정지선
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n"); 
    }
    else if(node_index >= 931 && node_index <= 947) //신호등, 직진, 947이 정지선
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n"); 
    }
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
    */
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
    ros::Subscriber load_identify         = nh.subscribe("/lane_mission",1,laneCallBack);

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
