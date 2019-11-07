#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <iostream>
#include <sstream>

float x,y,w,z;

void chatterCallback(const nav_msgs::Odometry msg)
{
  x = msg.pose.pose.position.x;
  y = msg.pose.pose.position.y;
  w = msg.pose.pose.orientation.w;
  z = msg.pose.pose.orientation.z;
  ROS_INFO("X %f",x);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "agv2_initial");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/agv1/odom", 1000, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
  ros::Time current_time;
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std::cout<<"x " <<x<<std::endl;
    current_time = ros::Time::now();
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;   
    msg.pose.pose.orientation.w = w;
    msg.pose.pose.orientation.z = z;

    if(x!=0){
       chatter_pub.publish(msg);
    }
    if(count>=5){
      ros::shutdown();
    }
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }


  ros::spin();

  return 0;
}
