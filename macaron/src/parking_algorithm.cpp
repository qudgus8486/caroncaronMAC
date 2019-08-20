#include "ros/ros.h"
#include <iostream>
#include "math.h"

#include "sensor_msgs/LaserScan.h"
#include "macaron/Floats.h"
#include "macaron/base_frame.h"
#include "macaron/erp42_write.h"


#define PI acos(-1)

//-------------------input value--------------------------//\


//from lanetraker
double deviation = 0;

//from lidar........??
double scan_data[2][180];
double carte_data[3][360];
double lane_data[2][23];
double cost[4][57];

//from gps_txt reader
double localized_p[2] = {0,0};   //현재 내 위치에서 가장 가까운 base path
double localized_vec;            //도북기준 각도, rad



//-------------------write value--------------------------//
bool write_E_stop;
int write_gear=0;
int write_speed=0;
int write_brake=1;
int write_steer=0;

//----------------control property--------------------------//
double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property

double alpha = 0;                // 현재 방향과 ld포인트 사이의 각도 차이 , rad
double ld = 2;                   //ld (m), 정수로 설정할 것
macaron::erp42_write erp42;



void laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    for(int i=0; i < 811; i++)
    {
        scan_data[0][i]=msg->ranges[i];
        scan_data[1][i]=0.3333*i-135;
        if(isinff(scan_data[0][i])) 
            scan_data[0][i] = 25;
    }
}


void laneCallBack(const macaron::Floats::ConstPtr& lane)
{
    lane->mid_point_vector;
    lane->testset.data;
    lane->testset.layout;
    deviation=lane->deviation;
    float dstride0 = lane->testset.layout.dim[0].stride;
    float dstride1 = lane->testset.layout.dim[1].stride;
    float h = lane->testset.layout.dim[0].size;
    float w = lane->testset.layout.dim[1].size;
    double l_cost=1;
    for (int i=0; i<h; i++)
    {
        for (int j=0; j<w; j++)
        {
            lane_data[i][j]=lane->testset.data[dstride1*i+j];
        }
    }

    double a=0;
    for (int j=0; j<23; j++) 
    {
        a+=lane_data[1][j];
    }

    //캘리브레이션 코드가 절실하다 빠른시일내에 작성하도록
}


void pathCallBack(const macaron::base_frame::ConstPtr& path) 
{
    localized_p[0]        = path->s_x[0];
    localized_p[1]        = path->s_y[0];
    localized_vec         = path->s_a[0];
}



void toERP42()
{
    write_steer = 71 * atan2(2*wheel_base*sin(-alpha), ld)*180/PI + 0.5; //conversion with rounding
    if(write_steer > 2000)
        write_steer = 2000;
    else if(write_steer < -2000)
        write_steer = -2000;

    erp42.write_speed = 30;
    erp42.write_steer = write_steer;
    erp42.write_gear  = 0;
    erp42.write_brake = 1;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"parking_algorithm");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher parking = nh.advertise<macaron::erp42_write>("/erp_write", 1);
    ros::Subscriber parking_lidar_sub   = nh.subscribe("/scan",100, laser_scan_Callback);
    ros::Subscriber parking_lane_sub    = nh.subscribe("/lane",10, laneCallBack);
    ros::Subscriber parking_gpspath_sub = nh.subscribe("/base_frame",10, pathCallBack);

    while(ros::ok)
    {
        ros::spinOnce();








        loop_rate.sleep();
    }
    return 0;
}
