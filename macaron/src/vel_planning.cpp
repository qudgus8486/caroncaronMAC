#include "ros/ros.h"
#include "math.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "macaron/base_frame.h"
#include "macaron/erp42_write.h"
#include "insgps_y/Message2.h"

#define PI acos(-1)

//----------------control property--------------------------//
double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
double ld = 7;                   //ld (m), 정수로 설정할 것
#define candidate_path_leng 9    //후보경로 s길이 (m), 정수로 설정할 것


//-------------------input value--------------------------//
//from path planner
double selected_path[2][int(candidate_path_leng) + 1];

//from gps_txt reader
double base_path[2][int(candidate_path_leng) + 1]  = { 0 };
double base_path_vec[int(candidate_path_leng) + 1] = { 0 };

//from Localization node
double x_tm;
double y_tm;
double heading; //도북기준 heading, rad

//from mission identifier
double speed_reduction = 1.0;

//-------------------write value--------------------------//
bool write_E_stop;
int write_gear  = 0;
int write_speed = 0;
int write_brake = 1;
int write_steer = 0;


void s_pathCallBack(const std_msgs::Float64MultiArray::ConstPtr& path)
{
    for(int xy = 0; xy < 2; xy++)
    {
        for(int i = 0; i <= int(candidate_path_leng); i++)
        {
            selected_path[xy][i] = path->data[i + (int(candidate_path_leng) + 1) * xy];
        }
    }
}


void headingCallBack(const insgps_y::Message2::ConstPtr& location)
{
    x_tm     = location->pos_x;
    y_tm     = location->pos_y;
    heading  = location->yaw;
}


void pathCallBack(const macaron::base_frame::ConstPtr& path) 
{
    for(int i = 0; i <= int(candidate_path_leng); i++)
    {
        base_path[0][i]  = path->s_x[i];
        base_path[1][i]  = path->s_y[i];
        base_path_vec[i] = path->s_a[i];
    }
}


void reductionCallBack(const std_msgs::Float64::ConstPtr& reduction) 
{
    speed_reduction = reduction->data;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"vel_planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher vel_planner = nh.advertise<macaron::erp42_write>("erp_write", 1);
    macaron::erp42_write erp42;

    ros::Subscriber seleted_path_sub    = nh.subscribe("/selected_path",10, s_pathCallBack);
    ros::Subscriber vel_localization    = nh.subscribe("/heading",10, headingCallBack);
    ros::Subscriber vel_baseframe_sub   = nh.subscribe("/base_frame",10, pathCallBack);
    ros::Subscriber speed_reduction_sub = nh.subscribe("/speed_reduction",10, reductionCallBack);

    while(ros::ok)
    {
        ros::spinOnce();
        
        double ld_x  = selected_path[0][int(ld)];
        double ld_y  = selected_path[1][int(ld)];
        double alpha = heading - atan2(y_tm - ld_y, x_tm - ld_x); //현재 방향과 ld포인트 사이의 각도 차이 , 오른손나사, rad

        //steer control
        write_steer = 71 * atan2(2*wheel_base*sin(alpha), ld)*180/PI + 0.5; //conversion with rounding
        if(write_steer > 2000)
            write_steer = 2000;
        else if(write_steer < -2000)
            write_steer = -2000;

        //velocity contrtol
        write_speed = int((2000.0 - fabs(write_steer)) / 2000.0 * (50 - 20) + 0.5) + 20;
        /*
        double kappa = fabs(base_path_vec[int(candidate_path_leng)] - heading);  //거리로 나누지 않은 곡률값 (단순히 각도 차이)
        if(kappa > 0.908)
            kappa = 0.908;
        write_speed = 20 / kappa;
        if(write_speed > 50) 
            write_speed = 50;   
        */

        erp42.write_speed = int(write_speed * speed_reduction + 0.5); //여기에 말고 최대속도에 곱하는 방안을 생각해보자.
        erp42.write_steer = write_steer;
        erp42.write_gear  = 0;
        erp42.write_brake = 1;
        vel_planner.publish(erp42);     
        
        printf("Selected steer angle : %fdegree\n",write_steer / 71.0);
        printf("Selected velocity    : %d\n\n",erp42.write_speed);
        printf("Speed reduction      : %f\n",speed_reduction);
        printf("---------------------------------------------------\n");
        loop_rate.sleep();
    }
    return 0;
}
