#include "ros/ros.h"
#include "tf/tf.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <iostream>
#include "eigen_conversion.h"
using namespace std;

/*  INITIALIZE USEFUL VARIABLES  */

float velocity_x = 0.0;
float velocity_y = 0.0;
float velocity_z = 0.0;

const float MIN_DIST_ALLOWED = 0.7; //closest distance allowed to an obstacle
const float STARTING_DIST = 10000; //initial distance between robot and obstacle
const float REGULATOR = 1 / 7000; //constant used to regulate the new velocity

geometry_msgs::Twist velocity_msg; //velocity obtained by the subscriber
geometry_msgs::Twist vel_updated; //updated velocity to be published

bool vel_detected = false;

ros::Publisher vel_pub;

/*DETECT THE DISTANCE FROM OBSTACLES USING THE LASER SCAN AND MODIFY THE VELOCITY COMMANDS TO AVOID COLLISIONS*/

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  if (!vel_detected) return;
  vel_detected = false;
  
  laser_geometry::LaserProjection laser_projection;
  sensor_msgs::PointCloud point_cloud;
  tf::TransformListener transform_listener;
  tf::StampedTransform stamped_transform;

  /*Transform a sensor_msgs::LaserScan into a sensor_msgs::PointCloud in target frame base_laser_link using transform_listener. So a 3D point cloud is obtained from a linear array*/
  laser_projection.transformLaserScanToPointCloud("base_laser_link", *scan_msg, point_cloud,transform_listener);

  try{
	  
    /*Wait until the transformation from base_laser_link to base_footprint is available */
    transform_listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(7,0));

    /*Get the transformation between the two coordinate frames, base_footprint and base_laser_link, saving the result into stamped_transform*/  
    transform_listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), stamped_transform);
  }

  catch(tf::TransformException &exception) {
    ROS_ERROR("%s", exception.what());
    return;
  }

  /* INITIALIZE THE REPULSIVE X, Y FORCES TO ZERO AND THE DISTANCE FROM THE NEAREST OBSTACLE*/
	
  float force_x = 0.0;
  float force_y = 0.0;
  
  float actual_obstacle_distance;
  float obstacle_distance = STARTING_DIST;

  /* Transform coordinates into a 2D matrix */
  Eigen::Isometry2f transform_matrix = convertPose2D(stamped_transform);  
  Eigen::Vector2f obstacle_pos, new_pos;

  /* Iterate over all the obstacles in point_cloud */
  for(auto& point: point_cloud.points){

    //compute position of the obstacle
    new_pos(0) = point.x;
    new_pos(1) = point.y;

    //transform the position into robot base frame
    new_pos = transform_matrix * new_pos;

    //compute the distance between the robot and the given obstacle
    actual_obstacle_distance = sqrt(pow(new_pos(0), 2) + pow(new_pos(1), 2));

    //determine the necessary forces
    force_x += new_pos(0) / pow(actual_obstacle_distance, 2);
    force_y += new_pos(1) / pow(actual_obstacle_distance, 2);

    //new closest obstacle? Then update the obstacle distance and its position
    if(actual_obstacle_distance < obstacle_distance){
      obstacle_distance = actual_obstacle_distance;
      obstacle_pos = new_pos;
    }
  }

  /*when the nearest obstacle is positioned at a distance lower than MIN_DIST_ALLOWED, regulate the forces and execute the update of the velocity. Otherwise just publish the unmodified velocity*/
  if (obstacle_distance < MIN_DIST_ALLOWED){

    cerr << "Too close to the obstacle: " << obstacle_distance << " meters" << endl;
    
    force_x = - force_x * REGULATOR;    
    force_y = - force_y * REGULATOR;
    
    vel_updated.linear.x =  velocity_x + force_x;
    vel_updated.linear.y =  velocity_y + force_y;
    
    if(obstacle_pos(1) > 0){
      vel_updated.angular.z = -1 / pow(obstacle_distance, 2);
    }
    
    else if (obstacle_pos(1) < 0){
      vel_updated.angular.z = 1 / pow(obstacle_distance, 2);   
    }
   
    vel_pub.publish(vel_updated);
  }
	
  else{
    cerr << "Robot in acceptable zone distance: " << obstacle_distance << " meters" << endl;
    vel_pub.publish(velocity_msg);
  }
  
}

/*DETECT THE ACTUAL VELOCITY OF THE ROBOT ALONG X,Y,Z AXES*/

void velocityScanCallback(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
  vel_detected = true;
  
  velocity_x = vel_msg->linear.x;
  velocity_y = vel_msg->linear.y;
  velocity_z = vel_msg->angular.z;
  
  velocity_msg = *vel_msg;

  cerr << "Actual velocity x= " << velocity_x << " , y= " << velocity_y << " , z= " << velocity_z << endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "my_obstacle_avoidance_node");
  
  ros::NodeHandle n;

  ros::Subscriber laser_sub = n.subscribe("/base_scan", 10, laserScanCallback);
  ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 1, velocityScanCallback);

  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  
  ros::spin();

  return 0;
}
