/**************************************************************************
 * This file is only useful for simulation; either used in tandem with    *
 * 'lidar3d_sim.launch' to rotate the virtual base (uncomment the define) *
 * below in that case), or with 'lidar3d_app.launch' as a stand-in for a  *
 * real motor if the motor hardware is not available.                     *
 * ***********************************************************************/ 

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdlib> 
#include <math.h>

#define PI 3.14159265359
#define D2R(deg)  (deg * (PI / 180.0))

// #define SIM  // Only uncomment if launching the 'lidar3d_sim.launch', it will publish to virtual joints.

int main(int argc, char **argv){
    #ifdef SIM
        std_msgs::Float64 j1[360];
        for(int j=0; j<360; j++)
            j1[j].data  = D2R(double(j));
    #else
        double j1[360];
        for(int j=0; j<360; j++)
            j1[j]  = D2R(double(j));
    #endif

    ros::init(argc, argv, "dummy_rotate_lidar3d");
    ros::NodeHandle n;

    #ifdef SIM
    ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/lidar3d/joint1_position_controller/command", 1000);
    #else
    ros::Publisher joint1_pub = n.advertise<geometry_msgs::Pose>("/base_rotation", 1000);
    #endif
    
    ros::Rate loop_rate(10);  // I think this is one of the things that dictate how often the point cloud updates

    int count = 0;
    while (ros::ok())
    {
        #ifdef SIM
        joint1_pub.publish(j1[count]);
        #else
        geometry_msgs::Pose base_pose;
        tf2::Quaternion q;
        q.setRPY(0.0,0.0,j1[count]);
        base_pose.orientation.x = q.x();
        base_pose.orientation.y = q.y();
        base_pose.orientation.z = q.z();
        base_pose.orientation.w = q.w();

        joint1_pub.publish(base_pose);
        #endif

        ros::spinOnce();
        loop_rate.sleep();
        if(count < 360)
            ++count;
        else
            count = 0;
    }
    return 0;
}