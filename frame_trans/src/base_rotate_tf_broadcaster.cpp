#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cstdlib> 
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

void poseCallback(const geometry_msgs::Pose);
Eigen::Matrix4f roty(double);

Eigen::Matrix4f A;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar3d_base_rotation_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/base_rotation", 1, &poseCallback);

  ros::spin();
  return 0;
}

Eigen::Matrix4f roty(double theta){
  Eigen::Matrix4f A;
  A << cos(theta) , 0 , sin(theta), 0
      ,0          , 1 , 0          , 0
      ,-sin(theta), 0 ,  cos(theta), 0
      ,0          , 0 , 0          , 1;
  return A;
}

void poseCallback(const geometry_msgs::Pose msg){
  Eigen::Matrix4f A;
  // A = roty(msg.angular.z);
  
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped trans_stamp;

  trans_stamp.header.stamp = ros::Time::now();
  trans_stamp.header.frame_id = "world";
  trans_stamp.child_frame_id = "base_link";
  // trans_stamp.transform.translation.x = A(0,3);
  // trans_stamp.transform.translation.y = A(1,3);
  // trans_stamp.transform.translation.z = A(2,3);
  // tf2::Quaternion q;
  // q.setRPY(
  //    atan2(A(2,1),A(2,2))
  //   ,atan2(-A(2,0),sqrt(pow(A(2,1),2)+pow(A(2,2),2)))
  //   ,atan2(A(1,0),A(0,0)));
  // trans_stamp.transform.rotation.x = q.x();
  // trans_stamp.transform.rotation.y = q.y();
  // trans_stamp.transform.rotation.z = q.z();
  // trans_stamp.transform.rotation.w = q.w();

  trans_stamp.transform.translation.x = msg.position.x;
  trans_stamp.transform.translation.y = msg.position.y;
  trans_stamp.transform.translation.z = msg.position.z;
  trans_stamp.transform.rotation.x = msg.orientation.x;
  trans_stamp.transform.rotation.y = msg.orientation.y;
  trans_stamp.transform.rotation.z = msg.orientation.z;
  trans_stamp.transform.rotation.w = msg.orientation.w;

  br.sendTransform(trans_stamp);
}