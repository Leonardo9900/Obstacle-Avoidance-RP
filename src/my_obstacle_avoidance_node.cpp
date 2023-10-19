#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
using namespace std;

void laserScanCallback(const sensor_msgs::LaserScan& msg)
{
  
  const std::vector<float>& ranges = msg.ranges;
  float min_distance = *std::min_element(ranges.begin(), ranges.end());
  
  cerr << "The minimum distance from the wall is " << min_distance << " meters" << endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "my_obstacle_avoidance_node");
  
  ros::NodeHandle n;

  ros::Subscriber laser_sub = n.subscribe("/base_scan", 10, laserScanCallback);
  
  ros::spin();

  return 0;
}
