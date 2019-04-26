#include "ros/ros.h"
#include "rosbag/bag.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
  
sensor_msgs::PointCloud cloud;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){

    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;

    if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
        return;
    }

    projector_.transformLaserScanToPointCloud("/base_link",*scan_in, cloud,listener_);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "scan1_to_cloud1_node");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/laser_scan1", 1, &scanCallback);
    ros::Publisher cloud1_pub = n.advertise<sensor_msgs::PointCloud>("/point_cloud1", 1000);

    ros::Rate loop_rate(100);
    int count = 0;
    while (ros::ok())
    {
        cloud1_pub.publish(cloud);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}