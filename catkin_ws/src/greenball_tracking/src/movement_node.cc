#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <PahoMQTT.hpp>
#include "geometry_msgs/Point.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

using namespace UNITREE_LEGGED_SDK;

// callback function for the subscriber
void ballCoordinatesCallback(const boost::shared_ptr<const geometry_msgs::Point>& msg, ros::Publisher pub)
{
  // print the x coordinate of the point
  ROS_INFO("x = %f", msg->x);

  // publish the received point
  pub.publish(*msg);
}

int main(int argc, char **argv)
{
  // initialize the ROS system and become a node
  ros::init(argc, argv, "ball_coordinates_reader");
  ros::NodeHandle nh;

  // create a publisher to the "/gotem" topic
  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("/gotem", 10);

  // create a boost function object that calls the ballCoordinatesCallback function with the right arguments
  boost::function<void(const boost::shared_ptr<const geometry_msgs::Point>&)> f =
      boost::bind(ballCoordinatesCallback, _1, pub);

  // create a subscriber to the "/ball_coordinates" topic
  ros::Subscriber sub = nh.subscribe("/ball_coordinates", 10, f);

  // enter a loop to process ROS callbacks
  ros::spin();

  return 0;
}
