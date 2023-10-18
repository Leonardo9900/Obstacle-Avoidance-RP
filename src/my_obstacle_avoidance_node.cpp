#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
using namespace std

void laserScanCallback(const sensor_msgs::LaserScan& msg)
{
  cerr << "I heard: as minimum range" << msg.range_min << endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "laser_listener");
  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/base_scan", 10, laserScanCallback);

  ros::spin();

  return 0;
}
