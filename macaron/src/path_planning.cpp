#include "ros/ros.h"
#include <iostream>
#include "math.h"

#include "sensor_msgs/LaserScan.h"
#include "macaron/Floats.h"
#include "macaron/base_frame.h"
#include "insgps_y/Message2.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

#define PI acos(-1)

//for visualization
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
visualization_msgs::Marker      path_rviz_msg;
visualization_msgs::MarkerArray path_viewer;
visualization_msgs::Marker      obstacle_rviz_msg;
visualization_msgs::MarkerArray obstacle_viewer;
visualization_msgs::Marker      lane_rviz_msg;
visualization_msgs::MarkerArray lane_viewer;
#define _8line_x 550922
#define _8line_y 199952
#define DEG2RAD  PI/180
//----------------control property--------------------------//
double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
double lidar_offset = 0;         //라이다가 전방에서 삐뚤어진 각도, rad, 반시계가 +값
double d_lidar_gps = 1;          //라이다와 gps사이의 거리.
#define candidate_num 6          //중심 제외 단방향으로의 후보경로       
double search_range  = 1;        //단방향 후보경로 탐색범위(m) ,q방향
#define candidate_path_leng 10   //후보경로의 절점개수
double path_leng = 7;            //후보경로의 길이 s길이 (m)
double lane_width = 3.5;         //차선 변경이 필요한 구간의 차선 폭, 예선 3.7, 본선 3.5

double w_offset      = 0.5;
double w_lane        = 0.0; 
double w_consistency = 0.1;
double w_obstacle    = 0.0;

double cost_offset;
double cost_lane;
double cost_consistency;
double cost_obstalce;
//-------------------input value--------------------------//
//from lanetraker
double lane_data[2][23];
double lane_abs[2][23];  //차선 중심의 절대좌표
int lane_data_size;      //lane_data 열의 크기
int seq_before = 0;
int lane_error = 1;

//from lidar
double scan_data[811];        //극좌표계(상대좌표계), 캘리브레이션 안되어있음
double obstacle_data[2][811]; //xy좌표계(절대좌표계), 캘리브레이션 되어있음

//from gps_txt reader
double base_path[2][int(candidate_path_leng)]      = { 0 };
double base_path_vec[int(candidate_path_leng)]     = { 0 };
double candidate_path[2][int(candidate_path_leng)] = { 0 };

double localized_p_before[2];   
double qi;                       //초기 오프셋
double qf = 0;                   //목표 오프셋
double qf_before;                //이전 목표 오프셋

//from Localization node
double x_tm;
double y_tm;
double heading; //도북기준 heading, rad

//from mission identifier
int available_path[3] = {0, 1, 0};

//-------------------write value--------------------------//
std_msgs::Float64MultiArray selected_path;

int firstrun = 0;
int selected_index;
int selected_lane;

//------------------obstacle value ----------------------------//
std_msgs::Float64MultiArray xy_data;
double x_obs[811];
double y_obs[811];


void ObstacleCallBack(const std_msgs::Float64MultiArray::ConstPtr& xy)
{
    int data_num = xy->data[0];
    for(int i =0 ; i<data_num ; i++)
    {
        x_obs[i] = xy->data[i+1];
        y_obs[i] = xy->data[i+data_num+1];
    }
}


void headingCallBack(const insgps_y::Message2::ConstPtr& location)
{
    heading  = location->yaw;
}


void laneCallBack(const macaron::Floats::ConstPtr& lane)
{
    int seq        = lane->header.seq;
    int dstride0   = lane->testset.layout.dim[0].stride;
    int dstride1   = lane->testset.layout.dim[1].stride;
    int h          = lane->testset.layout.dim[0].size;
    lane_data_size = lane->testset.layout.dim[1].size;
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < lane_data_size; j++)
        {
            lane_data[i][j] = lane->testset.data[dstride1*i+j];
        }
    }
    for(int i = 0; i < lane_data_size; i++)
    {
        lane_abs[0][i] = x_tm + lane_data[0][i]*cos(heading) + lane_data[1][i]*cos(heading - PI/2); //deviation은 내가 차선 오른쪽에 있을 때 + 이다.
        lane_abs[1][i] = y_tm + lane_data[0][i]*sin(heading) + lane_data[1][i]*sin(heading - PI/2);
    }
    if(seq == seq_before)
        lane_error = 1;
    else
        lane_error = 0;
    seq_before = seq;
    //차선을 제대로 읽지 못하고 흔들릴 때 이를 보정하는 코드가 필요, 퍼블리시 안할 때 seq 동일한지 확인 필요
}


void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i = 0; i < 811; i++)
    {
        scan_data[i] = scan->ranges[i]; 
        if(isinff(scan_data[i]) || scan_data[i] < 0.05) //측정값을 라이다의 측정가능 거리내로 필터링
            scan_data[i] = 25;
        double theta = (double(i) / 3.0 - 135) * PI / 180; //라이다 기준 실제 각도, rad
        obstacle_data[0][i] = x_tm + d_lidar_gps*cos(heading) + scan_data[i]*cos(heading + lidar_offset + theta);
        obstacle_data[1][i] = y_tm + d_lidar_gps*sin(heading) + scan_data[i]*sin(heading + lidar_offset + theta);
    }
}


void pathCallBack(const macaron::base_frame::ConstPtr& path) 
{
    localized_p_before[0] = base_path[0][0];
    localized_p_before[1] = base_path[1][0];
    qi                    = path->distance;
    path_leng             = path->ld;
    x_tm     = path->tm_x;
    y_tm     = path->tm_y;

    for(int i = 0; i < int(candidate_path_leng); i++)
    {
        base_path[0][i]  = path->s_x[i];
        base_path[1][i]  = path->s_y[i];
        base_path_vec[i] = path->s_a[i];
    }
}


void ablepathCallBack(const::std_msgs::Int32MultiArray::ConstPtr& lane_change)
{
    for(int i = 0; i < 3; i++)
    {
        available_path[i] = lane_change->data[i];
    }
}


void generate_candidate_path(int index, int lane)
{
    qf = search_range / double(candidate_num) * double(index) + (1 - lane) * lane_width;
    double theta = heading - base_path_vec[0];
    double ds = path_leng;
    double a1 = (ds * tan(theta) - 2 * (qf - qi)) / (ds * ds * ds);
    double a2 = (qf - qi) / (ds * ds);

    for(int i = 0; i < int(candidate_path_leng); i++)
    {
        double s = path_leng / double(candidate_path_leng - 1) * i;
        double offset = (s - ds)*(s - ds) * (a1*s - a2) + qf;
        candidate_path[0][i] = base_path[0][i] + offset * cos(base_path_vec[i] + PI/2);
        candidate_path[1][i] = base_path[1][i] + offset * sin(base_path_vec[i] + PI/2);
    }

    selected_path.data.resize(int(candidate_path_leng) * 2);
    for(int xy = 0; xy < 2; xy++)
    {
        for(int i = 0; i < int(candidate_path_leng); i++)
        {
             selected_path.data[i + int(candidate_path_leng) * xy] = candidate_path[xy][i];
        }
    }
}


double path_cost()
{
    //offset cost
	cost_offset = fabs(qf);

	//lane cost, if data set have an error, ignore it.
    double lane_d = 100; //임의의 큰 수, 후보경로의 끝점으로 부터 차선중심까지의 거리
    for(int i = 0; i < lane_data_size; i++)
    {
        double TEMP_d = sqrt(pow(candidate_path[0][int(candidate_path_leng) - 1] - lane_abs[0][i], 2) + pow(candidate_path[1][int(candidate_path_leng) - 1] - lane_abs[1][i], 2));
        if(TEMP_d < lane_d)
            lane_d = TEMP_d;
    }
    cost_lane = lane_d * (1 - lane_error);

	//consistency cost, IGNORE it on the First loop
	double common_s = path_leng - sqrt(pow(base_path[0][0] - localized_p_before[0], 2) + pow(base_path[1][0] - localized_p_before[1], 2));
	cost_consistency = fabs(qf - qf_before) * common_s * 0.5 * (1 - firstrun);

    //obstacle cost
    cost_obstalce = 0;
    for(int i = 0; i < 811; i++)
    {
        if(scan_data[i] < path_leng)
        {
            for(int k = 0; k < int(candidate_path_leng); k++)
            {
                double d_obstacle = sqrt(pow(candidate_path[0][k] - obstacle_data[0][i], 2) + pow(candidate_path[1][k] - obstacle_data[1][i], 2));
                if(d_obstacle < width)
                {
                    cost_obstalce = 1;
                    break;
                }
            }
            if(cost_obstalce == 1)
                break;
        }
    }

	return w_offset*cost_offset + w_lane*cost_lane + w_consistency*cost_consistency + w_obstacle*cost_obstalce;
	
}


void print_on_terminal()
{
    printf("Selected Lane             : %d\n",1 - selected_lane);
    printf("Selected Path Index       : %d\n",selected_index);
    printf("Selected Offset           : %fm\n\n",qf);
    printf("Selected Offset cost      : %f\n",cost_offset);
    printf("Selected Safety cost      : %f\n",cost_lane);
    printf("Selected Consistency cost : %f\n",cost_consistency);
    printf("Selected Obstalce cost    : %f\n",cost_obstalce);
    printf("---------------------------------------------------\n");
}


void visualizing_path(int index, int lane, int selected)
{
    for(int i = 0; i < int(candidate_path_leng); i++)
    {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;
        c.a = 1.0; c.r = 1.0; c.g = 1.0; c.b = 1.0 - 1 * selected;
        p.x = candidate_path[0][i] - _8line_x;
        p.y = candidate_path[1][i] - _8line_y;
        p.z = 0.0;
        path_rviz_msg.type = visualization_msgs::Marker::ARROW;    
        path_rviz_msg.scale.x = 0.3 + 0.4 * selected;
        path_rviz_msg.scale.y = 0.3 + 0.4 * selected;
        path_rviz_msg.scale.z = 0.1;
        path_rviz_msg.pose.position   = p;
        path_rviz_msg.color           = c;
        path_rviz_msg.header.frame_id = "map";
        path_rviz_msg.action = visualization_msgs::Marker::ADD;
        path_rviz_msg.id = i + 100 + (index + candidate_num) * int(candidate_path_leng) + lane * (2 * candidate_num + 1) * int(candidate_path_leng);
        path_viewer.markers.push_back(path_rviz_msg);
    }
}


void visualizing_obstacle()
{
    for(int i = 0; i < 811; i++)
    {
        if(scan_data[i] < candidate_path_leng)
        {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;
            c.a = 1.0; c.r = 1.0; c.g = 0.5; c.b = 0.0;
            p.x = obstacle_data[0][i] - _8line_x;
            p.y = obstacle_data[1][i] - _8line_y;
            p.z = 0.0;
            obstacle_rviz_msg.type = visualization_msgs::Marker::ARROW;    
            obstacle_rviz_msg.scale.x = 0.2;
            obstacle_rviz_msg.scale.y = 0.2;
            obstacle_rviz_msg.scale.z = 0.1;
            obstacle_rviz_msg.pose.position   = p;
            obstacle_rviz_msg.color           = c;
            obstacle_rviz_msg.header.frame_id = "map";
            obstacle_rviz_msg.action = visualization_msgs::Marker::ADD;
            obstacle_rviz_msg.id = i+500;
            obstacle_viewer.markers.push_back(obstacle_rviz_msg);
        }
        
    }
}


void visualizing_lane()
{
    for(int i = 0; i < lane_data_size; i++)
    {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;
        c.a = 1.0; c.r = 1.0; c.g = 0.0; c.b = 1.0;
        p.x = lane_abs[0][i] - _8line_x;
        p.y = lane_abs[1][i] - _8line_y;
        p.z = 0.0;
        lane_rviz_msg.type = visualization_msgs::Marker::ARROW;    
        lane_rviz_msg.scale.x = 0.6;
        lane_rviz_msg.scale.y = 0.6;
        lane_rviz_msg.scale.z = 0.1;
        lane_rviz_msg.pose.position   = p;
        lane_rviz_msg.color           = c ;
        lane_rviz_msg.header.frame_id = "map";
        lane_rviz_msg.action = visualization_msgs::Marker::ADD;
        lane_rviz_msg.id=i+1400;
        lane_viewer.markers.push_back(lane_rviz_msg);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"path_planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher path_planner        = nh.advertise<std_msgs::Float64MultiArray>("/selected_path", 10);
    ros::Publisher path_viewer_pub     = nh.advertise<visualization_msgs::MarkerArray>("/path_test",10);
    ros::Publisher obstacle_viewer_pub = nh.advertise<visualization_msgs::MarkerArray>("/obstacle",10);
    ros::Publisher lane_viewer_pub     = nh.advertise<visualization_msgs::MarkerArray>("/lane_viewer",10);

    ros::Subscriber localization       = nh.subscribe("/heading",10,headingCallBack);
    ros::Subscriber lane_sub           = nh.subscribe("/lane",10, laneCallBack);
    ros::Subscriber gpspath_sub        = nh.subscribe("/base_frame",10, pathCallBack);
    ros::Subscriber available_path_sub = nh.subscribe("/available_path", 10, ablepathCallBack);
    ros::Subscriber lidar_sub          = nh.subscribe("/scan",10,scanCallBack);
    ros::Subscriber obstacle_sub       = nh.subscribe("/obstacles",10,ObstacleCallBack);
    
    while(ros::ok)
    {
        ros::spinOnce();

        double TEMP = 100; //임의의 큰 수
        for(int i = 0; i < 3; i++)
        {
            if(available_path[i])
            {
                for(int index = -candidate_num; index <= candidate_num; index++)
                {
                    generate_candidate_path(index, i);
                    visualizing_path(index, i, 0);
                    if(path_cost() < TEMP)
                    {
                        TEMP = path_cost();
                        selected_index = index;
                        selected_lane = i;
                    }
                }
            }
        }
        if(firstrun)
            firstrun = 0;
        generate_candidate_path(selected_index, selected_lane);
        path_cost();
        visualizing_path(candidate_num + 1, 2, 1);
        qf_before = qf;
        print_on_terminal();
        visualizing_obstacle();
        visualizing_lane();
        path_viewer_pub.publish(path_viewer);
        path_viewer.markers.clear();
        obstacle_viewer_pub.publish(obstacle_viewer);
        obstacle_viewer.markers.clear();
        if(!lane_error)
        {
            lane_viewer_pub.publish(lane_viewer);
            lane_viewer.markers.clear();
        }

        path_planner.publish(selected_path);
        loop_rate.sleep();
    }
    return 0;
}
