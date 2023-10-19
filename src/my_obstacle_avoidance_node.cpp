#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
using namespace std;

float velocity_x = 0;
float velocity_y = 0;
float velocity_z = 0;

void laserScanCallback(const sensor_msgs::LaserScan& msg)
{
  
  const std::vector<float>& ranges = msg.ranges;
  float min_distance = *std::min_element(ranges.begin(), ranges.end());
  
  cerr << "The minimum distance from the wall is " << min_distance << " meters" << endl;
}

void velocityScanCallback(const geometry_msgs::Twist& vel_msg)
{

  velocity_x = vel_msg.linear.x;
  velocity_y = vel_msg.linear.y;
  velocity_z = vel_msg.angular.z;

  cerr << "Actual velocity x= " << velocity_x << " , y= " << velocity_y << " , z= " << velocity_z << endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "my_obstacle_avoidance_node");
  
  ros::NodeHandle n;

  ros::Subscriber laser_sub = n.subscribe("/base_scan", 10, laserScanCallback);
  ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 10, velocityScanCallback);

  ros::spin();

  return 0;
}
