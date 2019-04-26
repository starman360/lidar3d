#include "ros/ros.h"
#include "rosbag/bag.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_listener.h>
  
sensor_msgs::PointCloud cloud1, cloud2, cloud_total;

/**
 * \brief Function to merge the vectors of geometry_msgs::Point32
 * \param vin : input vector
 * \param vout : output vector (output value)
 */
void merge_point32_vector(const std::vector<geometry_msgs::Point32>& vin, std::vector<geometry_msgs::Point32>& vout)
{
    vout.reserve(vout.size() + vin.size());
    vout.insert(vout.end(), vin.begin(), vin.end());
}

/**
 * \brief Function to merge two PointCloud data after checking if they are in they have the same `frame_id`.
 * \param cloud_in1, cloud_in2 : Two input PointClouds
 * \param cloud_out : Output PointCloud
 * \return true if succesful and false otherwise
 */
bool merge_point_cloud(const sensor_msgs::PointCloud& cloud_in1, const sensor_msgs::PointCloud& cloud_in2, sensor_msgs::PointCloud& cloud_out)
{
    // check if both have the same frame_id
    if(cloud_in1.header.frame_id != cloud_in2.header.frame_id)
        return false;
    // set cloud_out frame_id
    cloud_out.header.frame_id = cloud_in1.header.frame_id;
    // merge the clouds one by one
    merge_point32_vector(cloud_in1.points, cloud_out.points);
    merge_point32_vector(cloud_in2.points, cloud_out.points);
    return true;
}

void cloud1_assembled_Callback (const sensor_msgs::PointCloud& cloud1_in){
    cloud1 = cloud1_in;
}

void cloud2_assembled_Callback (const sensor_msgs::PointCloud& cloud2_in){
    cloud2 = cloud2_in;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "cloud_aggregator_node");

    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/assembled_cloud1", 1, &cloud1_assembled_Callback);
    ros::Subscriber sub2 = n.subscribe("/assembled_cloud2", 1, &cloud2_assembled_Callback);
    ros::Publisher cloud1_pub = n.advertise<sensor_msgs::PointCloud>("/aggregated_cloud", 1000);

    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()){
        merge_point_cloud(cloud1, cloud2, cloud_total);
        cloud1_pub.publish(cloud_total);

        rosbag::Bag bag;
        bag.open("cloud.bag", rosbag::bagmode::Write);
        bag.write("cloud_data", ros::Time::now(), cloud_total);
        bag.close();

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}