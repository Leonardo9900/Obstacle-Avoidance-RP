#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
using namespace std;

float velocity_x = 0;
float velocity_y = 0;
float velocity_z = 0;

const float MIN_DIST_ALLOWED = 0.8;

geometry_msgs::Twist velocity_msg;
geometry_msgs::Twist vel_updated;

ros::Publisher vel_pub;

void laserScanCallback(const sensor_msgs::LaserScan& msg)
{
  
  const std::vector<float>& ranges = msg.ranges;
  float min_distance = *std::min_element(ranges.begin(), ranges.end());

  if(min_distance < MIN_DIST_ALLOWED){

    cerr << "The minimum distance from the wall is " << min_distance << " meters" << endl;
  	vel_updated.linear.x = 0.2;
  	vel_updated.linear.y = 0.2;
  	vel_updated.angular.z = 1.0;
  	
  	vel_pub.publish(vel_updated);
  }
  else{
  	cerr << "The distance is under control: " << min_distance << " meters" << endl;
    vel_pub.publish(velocity_msg);
  
  }
  
}

void velocityScanCallback(const geometry_msgs::Twist& vel_msg)
{

  velocity_msg = vel_msg;
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

  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  
  ros::spin();

  return 0;
}
