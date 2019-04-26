#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include "sensor_msgs/PointCloud.h"

namespace laser_assembler{

class PeriodicSnapshotter{

public:

  PeriodicSnapshotter()
  {
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud> ("assembled_cloud1", 1);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans>("assemble_scans");

    // Start the timer that will trigger the processing loop (timerCallback)
    timer_ = n_.createTimer(ros::Duration(5,0), &PeriodicSnapshotter::timerCallback, this);

    // Need to track if we've called the timerCallback at least once
    first_time_ = true;
  }

  void timerCallback(const ros::TimerEvent& e)
  {

    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (first_time_)
    {
      first_time_ = false;
      return;
    }

    // Populate our service request based on our timer callback times
    AssembleScans srv;
    srv.request.begin = e.last_real;
    srv.request.end   = e.current_real;

    // Make the service call
    if (client_.call(srv))
    {
      ROS_INFO("Published Cloud1 with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
      pub_.publish(srv.response.cloud);
    }
    else
    {
      ROS_ERROR("Error making service call\n") ;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
  ros::Timer timer_;
  bool first_time_;
};

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud1_periodic_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  PeriodicSnapshotter snapshotter;
  ros::spin();
  return 0;
}
