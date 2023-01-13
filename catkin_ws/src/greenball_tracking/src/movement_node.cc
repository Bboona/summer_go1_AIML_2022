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

ros::Publisher pub_high;

ros::Time last_message_time;

long motiontime = 0;

// Timer callback function
void checkMessageReceived(const ros::TimerEvent& event) 
{
    ros::Time current_time = ros::Time::now();
    cout<<"current time is: "<<current_time<<endl;
    cout<<"last_message_time time is: "<<last_message_time<<endl;
    if ((current_time - last_message_time).toSec() > 0.5){

        // No messages have been received for the past 0.5 second
        // Perform some action here

        unitree_legged_msgs::HighCmd high_cmd_stop;

        high_cmd_stop.head[0] = 0xFE;
        high_cmd_stop.head[1] = 0xEF;
        high_cmd_stop.levelFlag = HIGHLEVEL;
        high_cmd_stop.mode = 0;
        high_cmd_stop.gaitType = 0;
        high_cmd_stop.speedLevel = 0;
        high_cmd_stop.footRaiseHeight = 0;
        high_cmd_stop.bodyHeight = 0;
        high_cmd_stop.euler[0] = 0;
        high_cmd_stop.euler[1] = 0;
        high_cmd_stop.euler[2] = 0;
        high_cmd_stop.velocity[0] = 0.0f;
        high_cmd_stop.velocity[1] = 0.0f;
        high_cmd_stop.yawSpeed = 0.0f;
        high_cmd_stop.reserve = 0;

        pub_high.publish(high_cmd_stop);
    }
}

// callback function for the subscriber
void ballCoordinatesCallback(const geometry_msgs::Point::ConstPtr& msg)   
{
    last_message_time = ros::Time::now();

    unitree_legged_msgs::HighCmd high_cmd_ros;

    motiontime += 2;

    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.levelFlag = HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gaitType = 0;
    high_cmd_ros.speedLevel = 0;
    high_cmd_ros.footRaiseHeight = 0.08;
    high_cmd_ros.bodyHeight = 0;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yawSpeed = 0.0f;
    high_cmd_ros.reserve = 0;

    // Move left
    if(msg->x >= 0 && msg->x <= 165){
        cout<<"Moving left msg->x is: "<<msg->x<<endl;
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 1;
        high_cmd_ros.yawSpeed = 0.5f;
    }
    // Move forward
    else if(msg->x > 165 && msg->x < 330){
        cout<<"Moving forward msg->x is: "<<msg->x<<endl;
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 1;
        high_cmd_ros.velocity[0] = 0.1f;
    }
    // Move right
    else if(msg->x >= 330 && msg->x <= 460){
        cout<<"Moving right msg->x is: "<<msg->x<<endl;
        high_cmd_ros.mode = 2;
        high_cmd_ros.gaitType = 1;
        high_cmd_ros.yawSpeed = -0.5f;
    }

    pub_high.publish(high_cmd_ros);
}

int main(int argc, char **argv)
{
    // initialize the ROS system and become a node
    ros::init(argc, argv, "node_greenball_movement");
    ros::NodeHandle nh;

    // create a publisher to the "/high_state" topic
    pub_high = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

    // create a subscriber to the "/ball_coordinates" topic
    ros::Subscriber sub = nh.subscribe("/ball_coordinates", 1000, ballCoordinatesCallback);

    // Create a timer to check if messages are being received
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), checkMessageReceived);

    // enter a loop to process ROS callbacks
    ros::spin();

    return 0;
}
